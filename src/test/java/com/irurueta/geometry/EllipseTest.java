/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.Ellipse
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 19, 2017.
 */
package com.irurueta.geometry;

import com.irurueta.algebra.ArrayUtils;
import com.irurueta.statistics.UniformRandomizer;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class EllipseTest {        
    
    public static final double LARGE_ABSOLUTE_ERROR = 1e-3;
    public static final double ABSOLUTE_ERROR = 1e-6;
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final double MIN_RANDOM_DEGREES = -90.0;
    public static final double MAX_RANDOM_DEGREES = 90.0;
    
    public static final int TIMES = 500;
    
    public EllipseTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testConstructor() throws ColinearPointsException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            //test empty constructor
            Ellipse ellipse = new Ellipse();

            //check default values
            assertEquals(ellipse.getCenter(), new InhomogeneousPoint2D());
            assertEquals(ellipse.getSemiMajorAxis(), 1.0, 0.0);
            assertEquals(ellipse.getSemiMinorAxis(), 1.0, 0.0);
            assertEquals(ellipse.getRotationAngle(), 0.0, 0.0);

            //test constructor with center, axes and rotation
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            Point2D center = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            double semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                    MAX_RANDOM_VALUE);
            double semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                    semiMajorAxis);
            double rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                            MAX_RANDOM_DEGREES));
            ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, 
                    rotationAngle);

            //check correctness
            assertEquals(ellipse.getCenter(), center);
            assertEquals(ellipse.getSemiMajorAxis(), semiMajorAxis, 0.0);
            assertEquals(ellipse.getSemiMinorAxis(), semiMinorAxis, 0.0);
            assertEquals(ellipse.getRotationAngle(), rotationAngle, 0.0);
            
            ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis,
                    new Rotation2D(rotationAngle));

            //check correctness
            assertEquals(ellipse.getCenter(), center);
            assertEquals(ellipse.getSemiMajorAxis(), semiMajorAxis, 0.0);
            assertEquals(ellipse.getSemiMinorAxis(), semiMinorAxis, 0.0);
            assertEquals(ellipse.getRotationAngle(), rotationAngle, 0.0);            

            //test with 2 points, center and rotation angle
            double radius = Math.abs(randomizer.nextDouble(
                    MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE));

            Point2D point1, point2;
            boolean areEqual;
            do {
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

                //ensure that all three points are different
                areEqual = point1.equals(point2, ABSOLUTE_ERROR);
            }while(areEqual);

            ellipse = new Ellipse(point1, point2, center, rotationAngle);

            //check parameters
            assertEquals(ellipse.getCenter(), center);
            assertEquals(ellipse.getSemiMajorAxis(), radius, ABSOLUTE_ERROR);
            assertEquals(ellipse.getSemiMinorAxis(), radius, ABSOLUTE_ERROR);
            assertEquals(ellipse.getRotationAngle(), rotationAngle, 0.0);

            if (!ellipse.isLocus(point1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point1, ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point2, ABSOLUTE_ERROR));
            if (ellipse.isLocus(center, ABSOLUTE_ERROR)) {
                continue;
            }
            assertFalse(ellipse.isLocus(center, ABSOLUTE_ERROR));

            //test with 5 points (without threshold)
            radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                    MAX_RANDOM_VALUE));

            Point2D point3, point4, point5;
            do {
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
                angle = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                        MAX_RANDOM_DEGREES) * Math.PI / 180.0;
                point4 = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);
                angle = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                        MAX_RANDOM_DEGREES) * Math.PI / 180.0;
                point5 = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);

                //ensure that all three points are different
                areEqual = point1.equals(point2, ABSOLUTE_ERROR) ||
                        point2.equals(point3, ABSOLUTE_ERROR) ||
                        point3.equals(point1, ABSOLUTE_ERROR) ||
                        point4.equals(point1, ABSOLUTE_ERROR) ||
                        point5.equals(point1, ABSOLUTE_ERROR);
            }while(areEqual);

            try {
                ellipse = new Ellipse(point1, point2, point3, point4, point5);
            } catch (ColinearPointsException e) {
                continue;
            }

            //check parameters
            if (!ellipse.getCenter().equals(center, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.getCenter().equals(center, 
                    LARGE_ABSOLUTE_ERROR));
            if (Math.abs(ellipse.getSemiMajorAxis() - radius) > 
                    LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMajorAxis(), radius, 
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getSemiMinorAxis() - radius) > 
                    LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMinorAxis(), radius, 
                    LARGE_ABSOLUTE_ERROR);

            if(!ellipse.isLocus(point1, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point1, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point2, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point3, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point3, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point4, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point4, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point5, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point5, LARGE_ABSOLUTE_ERROR));
            assertFalse(ellipse.isLocus(center));

            ellipse = new Ellipse(point1, point2, point3, point4, point5, 
                    ABSOLUTE_ERROR);

            //check parameters
            assertTrue(ellipse.getCenter().equals(center, 
                    LARGE_ABSOLUTE_ERROR));
            assertEquals(ellipse.getSemiMajorAxis(), radius, 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(ellipse.getSemiMinorAxis(), radius, 
                    LARGE_ABSOLUTE_ERROR);

            if(!ellipse.isLocus(point1, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }        
            assertTrue(ellipse.isLocus(point1, LARGE_ABSOLUTE_ERROR));
            if(!ellipse.isLocus(point2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }        
            assertTrue(ellipse.isLocus(point2, LARGE_ABSOLUTE_ERROR));
            if(!ellipse.isLocus(point3, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }        
            assertTrue(ellipse.isLocus(point3, LARGE_ABSOLUTE_ERROR));
            if(!ellipse.isLocus(point4, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }        
            assertTrue(ellipse.isLocus(point4, LARGE_ABSOLUTE_ERROR));
            if(!ellipse.isLocus(point5, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }        
            assertTrue(ellipse.isLocus(point5, LARGE_ABSOLUTE_ERROR));
            assertFalse(ellipse.isLocus(center));

            //constructor with parameters
            double a = ellipse.getA();
            double b = ellipse.getB();
            double c = ellipse.getC();
            double d = ellipse.getD();
            double e = ellipse.getE();
            double f = ellipse.getF();

            Ellipse ellipse2 = new Ellipse(a, b, c, d, e, f);

            //check correctness
            assertTrue(ellipse.getCenter().equals(ellipse2.getCenter(), 
                    ABSOLUTE_ERROR));
            if (Math.abs(ellipse.getSemiMajorAxis() - 
                    ellipse2.getSemiMajorAxis()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMajorAxis(), 
                    ellipse2.getSemiMajorAxis(), ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getSemiMinorAxis() - 
                    ellipse2.getSemiMinorAxis()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMinorAxis(), 
                    ellipse2.getSemiMinorAxis(), ABSOLUTE_ERROR);


            ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, 
                    rotationAngle);

            a = ellipse.getA();
            b = ellipse.getB();
            c = ellipse.getC();
            d = ellipse.getD();
            e = ellipse.getE();
            f = ellipse.getF();

            ellipse2 = new Ellipse(a, b, c, d, e, f, ABSOLUTE_ERROR);

            //check correctness
            assertTrue(ellipse.getCenter().equals(ellipse2.getCenter(), 
                    ABSOLUTE_ERROR));
            if (Math.abs(ellipse.getSemiMajorAxis() - 
                    ellipse2.getSemiMajorAxis()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMajorAxis(), 
                    ellipse2.getSemiMajorAxis(), ABSOLUTE_ERROR);
            assertEquals(ellipse.getSemiMinorAxis(), 
                    ellipse2.getSemiMinorAxis(), ABSOLUTE_ERROR);

            if (Math.abs(ellipse.getRotationAngle() - 
                    ellipse2.getRotationAngle()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getRotationAngle(), 
                    ellipse2.getRotationAngle(), ABSOLUTE_ERROR);    

            Conic conic = ellipse.toConic();
            
            ellipse2 = new Ellipse(conic);
            
            //check correctness
            if(ellipse.getCenter().distanceTo(ellipse2.getCenter()) > 
                    ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getCenter().distanceTo(ellipse2.getCenter()), 
                    0.0, ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getSemiMajorAxis() - ellipse2.getSemiMajorAxis()) > 
                    ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMajorAxis(), 
                    ellipse2.getSemiMajorAxis(), ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getSemiMinorAxis() - ellipse2.getSemiMinorAxis()) > 
                    ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMinorAxis(), 
                    ellipse2.getSemiMinorAxis(), ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getRotationAngle() - ellipse2.getRotationAngle()) > 
                    ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getRotationAngle(), 
                    ellipse2.getRotationAngle(), ABSOLUTE_ERROR);

            //Force IllegalArgumentException
            ellipse2 = null;
            conic = new Conic();
            try {
                ellipse2 = new Ellipse(conic);
                fail("IllegalArgumentException expected but not thrown");
            } catch (IllegalArgumentException ex) { }
            
            assertNull(ellipse2);
            
            
            Circle circle = new Circle(center, radius);
            ellipse = new Ellipse(circle);
            
            //check correctness
            assertEquals(circle.getCenter(), ellipse.getCenter());
            assertEquals(circle.getRadius(), ellipse.getSemiMajorAxis(), 0.0);
            assertEquals(circle.getRadius(), ellipse.getSemiMinorAxis(), 0.0);
            assertEquals(ellipse.getRotationAngle(), 0.0, 0.0);
            
            numValid++;
        }
        
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testGetSetCenter() {
        Ellipse ellipse = new Ellipse();
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        //new value
        ellipse.setCenter(center);
        
        //check correctness
        assertEquals(ellipse.getCenter(), center);
    }
    
    @Test
    public void testGetSetSemiMajorAxis() {
        Ellipse ellipse = new Ellipse();
        
        //initial value
        assertEquals(ellipse.getSemiMajorAxis(), 1.0, 0.0);
        
        //new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE);

        ellipse.setSemiMajorAxis(semiMajorAxis);
        
        //check correctness
        assertEquals(ellipse.getSemiMajorAxis(), semiMajorAxis, 0.0);
    }
    
    @Test
    public void testGetSetSemiMinorAxis() {
        Ellipse ellipse = new Ellipse();
        
        //initial value
        assertEquals(ellipse.getSemiMinorAxis(), 1.0, 0.0);
        
        //new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0,
                MAX_RANDOM_VALUE);
        
        ellipse.setSemiMinorAxis(semiMinorAxis);
        
        //check correctness
        assertEquals(ellipse.getSemiMinorAxis(), semiMinorAxis, 0.0);
    }
    
    @Test
    public void testGetSetRotationAngle() {
        Ellipse ellipse = new Ellipse();
        
        //initial value
        assertEquals(ellipse.getRotationAngle(), 0.0, 0.0);
        
        //new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        
        ellipse.setRotationAngle(rotationAngle);
        
        //check correctness
        assertEquals(ellipse.getRotationAngle(), rotationAngle, 0.0);
    }
    
    @Test
    public void testGetSetRotation() {
        Ellipse ellipse = new Ellipse();
        
        //initial value
        assertEquals(ellipse.getRotation().getTheta(), 0.0, 0.0);
        
        //new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        
        Rotation2D rotation = new Rotation2D(rotationAngle);
        ellipse.setRotation(rotation);
        
        //check correctness
        assertEquals(ellipse.getRotation().getTheta(), rotationAngle, 0.0);
        assertEquals(ellipse.getRotationAngle(), rotationAngle, 0.0);
    }
    
    @Test
    public void testGetSetA() {
        Ellipse ellipse = new Ellipse();
        
        assertEquals(ellipse.getA(), 1.0, ABSOLUTE_ERROR);
        
        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double a = randomizer.nextDouble(ABSOLUTE_ERROR, MAX_RANDOM_VALUE);
        
        ellipse.setA(a);
        
        //check correctness
        assertEquals(ellipse.getA(), 1.0, ABSOLUTE_ERROR);
    }
    
    @Test
    public void testGetSetB() {
        Ellipse ellipse = new Ellipse();
        
        assertEquals(ellipse.getB(), 0.0, ABSOLUTE_ERROR);
        
        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double b = randomizer.nextDouble(0.0, 
                Math.sqrt(4.0*ellipse.getA()*ellipse.getC()));
        
        ellipse.setB(b);
        
        //check correctness
        assertTrue(Math.abs(ellipse.getB()) > ABSOLUTE_ERROR);
    }
    
    @Test
    public void testGetSetC() {
        Ellipse ellipse = new Ellipse();
        
        assertEquals(ellipse.getC(), 1.0, ABSOLUTE_ERROR);
        
        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double c = randomizer.nextDouble(ellipse.getB()*ellipse.getB() / 
                (4.0*ellipse.getA()), MAX_RANDOM_VALUE);
        
        ellipse.setC(c);
        
        //check correctness
        assertEquals(ellipse.getC(), 1.0, ABSOLUTE_ERROR);
    }
    
    @Test
    public void testGetSetD() {
        Ellipse ellipse = new Ellipse();
        
        assertEquals(ellipse.getD(), 0.0, ABSOLUTE_ERROR);
        
        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        ellipse.setD(d);
        
        //check correctness
        assertTrue(Math.abs(ellipse.getD()) > ABSOLUTE_ERROR);
    }
    
    @Test
    public void testGetSetE() {
        Ellipse ellipse = new Ellipse();
        
        assertEquals(ellipse.getE(), 0.0, ABSOLUTE_ERROR);
        
        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        ellipse.setE(e);
        
        //check correctness
        assertTrue(Math.abs(ellipse.getE()) > ABSOLUTE_ERROR);
    }    
    
    @Test
    public void testGetSetF() {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            Ellipse ellipse = new Ellipse();
        
            assertEquals(ellipse.getF(), -1.0, ABSOLUTE_ERROR);
        
            //set new value
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double f = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
        
            ellipse.setF(f);
        
            if (Math.abs(ellipse.getF() + 1.0) > ABSOLUTE_ERROR) {
                numValid++;
            }
        }
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testSetCenterAxesAndRotationAngle() {
        Ellipse ellipse = new Ellipse();
        
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
               randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE);
        double semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                semiMajorAxis);
        double rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                        MAX_RANDOM_DEGREES));
        
        ellipse.setCenterAxesAndRotation(center, semiMajorAxis, semiMinorAxis, 
                rotationAngle);
        
        //check correctness
        assertEquals(ellipse.getCenter(), center);
        assertEquals(ellipse.getSemiMajorAxis(), semiMajorAxis, 0.0);
        assertEquals(ellipse.getSemiMinorAxis(), semiMinorAxis, 0.0);
        assertEquals(ellipse.getRotationAngle(), rotationAngle, 0.0);        
    }

    @Test
    public void testSetCenterAxesAndRotation() {
        Ellipse ellipse = new Ellipse();
        
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
               randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE);
        double semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                semiMajorAxis);
        double rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                        MAX_RANDOM_DEGREES));
        Rotation2D rotation = new Rotation2D(rotationAngle);
        
        ellipse.setCenterAxesAndRotation(center, semiMajorAxis, semiMinorAxis, 
                rotation);
        
        //check correctness
        assertEquals(ellipse.getCenter(), center);
        assertEquals(ellipse.getSemiMajorAxis(), semiMajorAxis, 0.0);
        assertEquals(ellipse.getSemiMinorAxis(), semiMinorAxis, 0.0);
        assertEquals(ellipse.getRotationAngle(), rotationAngle, 0.0);        
    }
    
    @Test
    public void testSetParametersFromPointsCenterAndRotation() 
            throws ColinearPointsException {
        for (int t = 0; t < TIMES; t++) {
            Ellipse ellipse = new Ellipse();

            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double radius = Math.abs(randomizer.nextDouble(
                    MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE));

            Point2D center = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            double rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));

            Point2D point1, point2;
            boolean areEqual;
            do {
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

                //ensure that all three points are different
                areEqual = point1.equals(point2, ABSOLUTE_ERROR);
            } while (areEqual);

            ellipse.setParametersFromPointsCenterAndRotation(point1, point2, center,
                    rotationAngle);

            //check parameters
            assertEquals(ellipse.getCenter(), center);
            assertEquals(ellipse.getSemiMajorAxis(), radius, ABSOLUTE_ERROR);
            assertEquals(ellipse.getSemiMinorAxis(), radius, ABSOLUTE_ERROR);
            assertEquals(ellipse.getRotationAngle(), rotationAngle, 0.0);
            if (!ellipse.isLocus(point1, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point1, ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point2, ABSOLUTE_ERROR));
            if (ellipse.isLocus(center, ABSOLUTE_ERROR)) {
                continue;
            }
            assertFalse(ellipse.isLocus(center, ABSOLUTE_ERROR));
        }
    }
    
    @Test
    public void testSetParametersFromPoints() throws ColinearPointsException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            Ellipse ellipse = new Ellipse();

            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                    MAX_RANDOM_VALUE));
            Point2D center = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

            Point2D point1, point2, point3, point4, point5;
            boolean areEqual;
            do {
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
                angle = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                        MAX_RANDOM_DEGREES) * Math.PI / 180.0;
                point4 = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);
                angle = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                        MAX_RANDOM_DEGREES) * Math.PI / 180.0;
                point5 = new HomogeneousPoint2D(
                        center.getInhomX() + radius * Math.cos(angle),
                        center.getInhomY() + radius * Math.sin(angle), 1.0);

                //ensure that all three points are different
                areEqual = point1.equals(point2, ABSOLUTE_ERROR) ||
                        point2.equals(point3, ABSOLUTE_ERROR) ||
                        point3.equals(point1, ABSOLUTE_ERROR) ||
                        point4.equals(point1, ABSOLUTE_ERROR) ||
                        point5.equals(point1, ABSOLUTE_ERROR);
            }while(areEqual);

            try {
                ellipse.setParametersFromPoints(point1, point2, point3, point4, 
                        point5);
            } catch (ColinearPointsException e) {
                continue;
            }
        
            //check parameters
            if (!ellipse.getCenter().equals(center, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.getCenter().equals(center, 
                    LARGE_ABSOLUTE_ERROR));
            if (Math.abs(ellipse.getSemiMajorAxis() - radius) > 
                    LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMajorAxis(), radius, 
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getSemiMinorAxis() - radius) > 
                    LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMinorAxis(), radius, 
                    LARGE_ABSOLUTE_ERROR);

            if(!ellipse.isLocus(point1, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point1, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point2, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point3, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point3, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point4, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point4, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point5, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point5, LARGE_ABSOLUTE_ERROR));
            assertFalse(ellipse.isLocus(center));
            
            
            //use threshold
            ellipse.setParametersFromPoints(point1, point2, point3, point4, 
                    point5, ABSOLUTE_ERROR);
        
            //check parameters
            if (!ellipse.getCenter().equals(center, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.getCenter().equals(center, 
                    LARGE_ABSOLUTE_ERROR));
            if (Math.abs(ellipse.getSemiMajorAxis() - radius) > 
                    LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMajorAxis(), radius, 
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getSemiMinorAxis() - radius) > 
                    LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMinorAxis(), radius, 
                    LARGE_ABSOLUTE_ERROR);

            if(!ellipse.isLocus(point1, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point1, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point2, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point3, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point3, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point4, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point4, LARGE_ABSOLUTE_ERROR));
            if (!ellipse.isLocus(point5, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse.isLocus(point5, LARGE_ABSOLUTE_ERROR));
            assertFalse(ellipse.isLocus(center));
            
            numValid++;
        }

        assertTrue(numValid > 0);
    }
    
    @Test
    public void testSetParameters() {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            Point2D center = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            double semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                    MAX_RANDOM_VALUE);
            double semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                    MAX_RANDOM_VALUE);
            double rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                            MAX_RANDOM_DEGREES));
            
            Ellipse ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, 
                    rotationAngle);
            
            double a = ellipse.getA();
            double b = ellipse.getB();
            double c = ellipse.getC();
            double d = ellipse.getD();
            double e = ellipse.getE();
            double f = ellipse.getF();

            Ellipse ellipse2 = new Ellipse();
            ellipse2.setParameters(a, b, c, d, e, f);
            
            //check correctness
            assertTrue(ellipse.getCenter().equals(ellipse2.getCenter(), 
                    ABSOLUTE_ERROR));
            if (Math.abs(ellipse.getSemiMajorAxis() - 
                    ellipse2.getSemiMajorAxis()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMajorAxis(), 
                    ellipse2.getSemiMajorAxis(), ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getSemiMinorAxis() - 
                    ellipse2.getSemiMinorAxis()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMinorAxis(), 
                    ellipse2.getSemiMinorAxis(), ABSOLUTE_ERROR);
            
            //test again with threshold
            ellipse2.setParameters(a, b, c, d, e, f, ABSOLUTE_ERROR);
            
            //check correctness
            assertTrue(ellipse.getCenter().equals(ellipse2.getCenter(), 
                    ABSOLUTE_ERROR));
            if (Math.abs(ellipse.getSemiMajorAxis() - 
                    ellipse2.getSemiMajorAxis()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMajorAxis(), 
                    ellipse2.getSemiMajorAxis(), ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getSemiMinorAxis() - 
                    ellipse2.getSemiMinorAxis()) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMinorAxis(), 
                    ellipse2.getSemiMinorAxis(), ABSOLUTE_ERROR);

            numValid++;
        }
        
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testGetFocus() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE);
        double semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                semiMajorAxis);
        double rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                        MAX_RANDOM_DEGREES));
            
        Ellipse ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, 
                rotationAngle);
        
        assertEquals(ellipse.getFocus(), Math.sqrt(
                Math.pow(semiMajorAxis, 2.0) - Math.pow(semiMinorAxis, 2.0)),
                ABSOLUTE_ERROR);        
    }
    
    @Test
    public void testGetSetSemiMajorAxisCoordinates() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE);
        double semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                semiMajorAxis);
        double rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                        MAX_RANDOM_DEGREES));
            
        Ellipse ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, 
                rotationAngle);

        double[] coords1 = new double[2];
        ellipse.getSemiMajorAxisCoordinates(coords1);
        
        double[] coords2 = ellipse.getSemiMajorAxisCoordinates();
        
        assertArrayEquals(coords1, coords2, ABSOLUTE_ERROR);
        
        assertEquals(coords1[0], semiMajorAxis*Math.cos(rotationAngle),
                ABSOLUTE_ERROR);
        assertEquals(coords1[1], semiMajorAxis*Math.sin(rotationAngle),
                ABSOLUTE_ERROR);
        
        //semi major and semi minor coordinates are orthogonal
        assertEquals(ArrayUtils.dotProduct(coords1, 
                ellipse.getSemiMinorAxisCoordinates()), 0.0, ABSOLUTE_ERROR);
        
        
        Ellipse ellipse2 = new Ellipse();
        ellipse2.setSemiMajorAxisCoordinates(coords1);
        
        assertArrayEquals(ellipse2.getSemiMajorAxisCoordinates(), coords1, 
                ABSOLUTE_ERROR);
    }
    
    @Test
    public void testGetSetSemiMinorAxisCoordinates() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE);
        double semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                semiMajorAxis);
        double rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                        MAX_RANDOM_DEGREES));
            
        Ellipse ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, 
                rotationAngle);

        double[] coords1 = new double[2];
        ellipse.getSemiMinorAxisCoordinates(coords1);
        
        double[] coords2 = ellipse.getSemiMinorAxisCoordinates();
        
        assertArrayEquals(coords1, coords2, ABSOLUTE_ERROR);
        
        assertEquals(coords1[0], -semiMinorAxis*Math.sin(rotationAngle),
                ABSOLUTE_ERROR);
        assertEquals(coords1[1], semiMinorAxis*Math.cos(rotationAngle),
                ABSOLUTE_ERROR);
        
        //semi major and semi minor coordinates are orthogonal
        assertEquals(ArrayUtils.dotProduct(
                ellipse.getSemiMajorAxisCoordinates(), coords1), 0.0, 
                ABSOLUTE_ERROR);
        
        Ellipse ellipse2 = new Ellipse();
        ellipse2.setSemiMinorAxisCoordinates(coords1);
        
        assertArrayEquals(ellipse2.getSemiMinorAxisCoordinates(), coords1,
                ABSOLUTE_ERROR);
    }
    
    @Test
    public void testGetSetFocusPoints() {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            Point2D center = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            double semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                    MAX_RANDOM_VALUE);
            double semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                    semiMajorAxis);
            double rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                            MAX_RANDOM_DEGREES));

            Ellipse ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, 
                    rotationAngle);

            double focus = ellipse.getFocus();

            Point2D p1a = Point2D.create();
            ellipse.getFocusPoint1(p1a);

            Point2D p1b = ellipse.getFocusPoint1();

            assertEquals(p1a.distanceTo(center), focus, ABSOLUTE_ERROR);
            assertEquals(p1b.distanceTo(center), focus, ABSOLUTE_ERROR);

            Point2D p2a = Point2D.create();
            ellipse.getFocusPoint2(p2a);

            Point2D p2b = ellipse.getFocusPoint2();

            assertEquals(p2a.distanceTo(center), focus, ABSOLUTE_ERROR);
            assertEquals(p2b.distanceTo(center), focus, ABSOLUTE_ERROR);

            Line2D line = new Line2D(p1a, p2a);
            assertTrue(line.isLocus(center, ABSOLUTE_ERROR));


            Ellipse ellipse2 = new Ellipse(center, semiMajorAxis, semiMinorAxis, 
                    0.0);

            ellipse2.setFocusPoints(p1a, p2a, true);

            assertEquals(ellipse2.getSemiMinorAxis(), semiMinorAxis, 
                    ABSOLUTE_ERROR);
            if (!ellipse2.getFocusPoint1().equals(p1a, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse2.getFocusPoint1().equals(p1a, LARGE_ABSOLUTE_ERROR));
            if (!ellipse2.getFocusPoint2().equals(p2a, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse2.getFocusPoint2().equals(p2a, LARGE_ABSOLUTE_ERROR));        


            Ellipse ellipse3 = new Ellipse(center, semiMajorAxis, semiMinorAxis,
                    0.0);

            ellipse3.setFocusPoints(p1a, p2a, false);

            assertEquals(ellipse3.getSemiMajorAxis(), semiMajorAxis,
                    ABSOLUTE_ERROR);
            if (!ellipse3.getFocusPoint1().equals(p1a, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse3.getFocusPoint1().equals(p1a, LARGE_ABSOLUTE_ERROR));
            if (!ellipse3.getFocusPoint2().equals(p2a, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ellipse3.getFocusPoint2().equals(p2a, LARGE_ABSOLUTE_ERROR));
        
            numValid++;
        }
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testGetEccentricity() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE);
        double semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                semiMajorAxis);
        double rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                        MAX_RANDOM_DEGREES));
            
        Ellipse ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, 
                rotationAngle);

        double focus = ellipse.getFocus();

        assertEquals(ellipse.getEccentricity(), focus / semiMajorAxis, 
                ABSOLUTE_ERROR);
    }
    
    @Test
    public void testGetArea() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE);
        double semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE);
        double rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                        MAX_RANDOM_DEGREES));
            
        Ellipse ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, 
                rotationAngle);

        assertEquals(ellipse.getArea(), Math.PI * semiMajorAxis * semiMinorAxis,
                ABSOLUTE_ERROR);
        
        double radius = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE);
        Circle c = new Circle(center, radius);
        
        ellipse.setFromConic(c.toConic());
        
        assertEquals(ellipse.getArea(), c.getArea(), ABSOLUTE_ERROR);
    }
    
    @Test    
    public void testGetPerimeter() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE);
        double semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE);
        double rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                        MAX_RANDOM_DEGREES));
            
        Ellipse ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, 
                rotationAngle);

        double a = semiMajorAxis;
        double b = semiMinorAxis;
        assertEquals(ellipse.getPerimeter(), 
                Math.PI * (3.0 * (a + b) - Math.sqrt((3.0*a + b) * (a + 3.0*b))),
                ABSOLUTE_ERROR);
        
        double radius = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE);
        Circle c = new Circle(center, radius);
        
        ellipse.setFromConic(c.toConic());

        assertEquals(ellipse.getPerimeter(), c.getPerimeter(), ABSOLUTE_ERROR);
    }
    
    @Test
    public void testGetCurvature() {
        //build an ellipse like a circle
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));        
        double radius = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE);
        Ellipse ellipse = new Ellipse(center, radius, radius, 0.0);
        
        
        double angle = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        Point2D point1 = new HomogeneousPoint2D(
                center.getInhomX() + radius * Math.cos(angle),
                center.getInhomY() + radius * Math.sin(angle), 1.0);

        assertTrue(ellipse.isLocus(point1, ABSOLUTE_ERROR));
        
        //curvature for a circle is equal to the reciprocal of its radius
        assertEquals(ellipse.getCurvature(point1), 1.0 / radius, 
                ABSOLUTE_ERROR);
    }
    
    @Test
    public void testToConic() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new HomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 1.0);
        double radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE));
        double theta = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        center.normalize();

        Ellipse ellipse = new Ellipse(center, radius, radius, 0.0);
        Circle circle =  new Circle(center, radius);
        Conic conic = ellipse.toConic();
        Conic conic2 = circle.toConic();
        
        conic.normalize();
        conic2.normalize();
        
        //center is not locus
        assertFalse(ellipse.isLocus(center));
        assertFalse(ellipse.isLocus(center, ABSOLUTE_ERROR));
        assertFalse(conic.isLocus(center));
        assertFalse(conic.isLocus(center, ABSOLUTE_ERROR));
        
        Point2D locus = new HomogeneousPoint2D(
                center.getInhomX() + radius * Math.cos(theta),
                center.getInhomY() + radius * Math.sin(theta), 1.0); 
        locus.normalize();
        
        //test is locus
        assertTrue(ellipse.isLocus(locus, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(locus, ABSOLUTE_ERROR));
        
        //check correctness of estimated conic matrix (up to scale)
        assertTrue(conic.asMatrix().equals(conic2.asMatrix(), ABSOLUTE_ERROR));
    }
    
    @Test
    public void testSetFromConic() {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            Point2D center = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            double semiMajorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                    MAX_RANDOM_VALUE);
            double semiMinorAxis = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                    MAX_RANDOM_VALUE);
            double rotationAngle = com.irurueta.geometry.Utils.convertToRadians(
                    randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES));
            
            Ellipse ellipse = new Ellipse(center, semiMajorAxis, semiMinorAxis, 
                    rotationAngle);
        
            //test from conic
            Conic conic = ellipse.toConic();
            Ellipse ellipse2 = new Ellipse();
            ellipse2.setFromConic(conic);
        
            //check correctness
            if(ellipse.getCenter().distanceTo(ellipse2.getCenter()) > 
                    ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getCenter().distanceTo(ellipse2.getCenter()), 
                    0.0, ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getSemiMajorAxis() - ellipse2.getSemiMajorAxis()) > 
                    ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMajorAxis(), 
                    ellipse2.getSemiMajorAxis(), ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getSemiMinorAxis() - ellipse2.getSemiMinorAxis()) > 
                    ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getSemiMinorAxis(), 
                    ellipse2.getSemiMinorAxis(), ABSOLUTE_ERROR);
            if (Math.abs(ellipse.getRotationAngle() - ellipse2.getRotationAngle()) > 
                    ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(ellipse.getRotationAngle(), 
                    ellipse2.getRotationAngle(), ABSOLUTE_ERROR);
                        
            //Force IllegalArgumentException
            conic = new Conic();
            try{
                ellipse2.setFromConic(conic);
                fail("IllegalArgumentException expected but not thrown");
            }catch(IllegalArgumentException e){ }
            
            numValid++;
        }     
        
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testSetFromCircle() {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            Point2D center = new InhomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            double radius = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                    MAX_RANDOM_VALUE);
            
            Circle circle = new Circle(center, radius);
            
            Ellipse ellipse = new Ellipse();
            ellipse.setFromCircle(circle);
            
            //check correctness
            assertEquals(circle.getCenter(), ellipse.getCenter());
            assertEquals(circle.getRadius(), ellipse.getSemiMajorAxis(), 0.0);
            assertEquals(circle.getRadius(), ellipse.getSemiMinorAxis(), 0.0);
            assertEquals(ellipse.getRotationAngle(), 0.0, 0.0);
            
            numValid++;
        }
        
        assertEquals(numValid, TIMES);
    }
    
    @Test
    public void testIsInside() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D center = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE));
        double value = randomizer.nextDouble(0.2, 0.8);
        double value2 = 1.0 + value;
        double theta = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;

        Ellipse ellipse = new Ellipse(center, radius, radius, 0.0);
        
        Point2D inside = new InhomogeneousPoint2D(
                center.getInhomX() + value * radius * Math.cos(theta),
                center.getInhomY() + value * radius * Math.sin(theta));
        Point2D outside = new InhomogeneousPoint2D(
                center.getInhomX() + value2 * radius * Math.cos(theta),
                center.getInhomY() + value2 * radius * Math.sin(theta));
        
        //check correctness
        assertTrue(ellipse.isInside(inside));
        assertTrue(ellipse.isInside(inside, ABSOLUTE_ERROR));        
        
        assertFalse(ellipse.isInside(outside));
        assertFalse(ellipse.isInside(outside, ABSOLUTE_ERROR));
        
        //test for a large positive threshold 
        assertTrue(ellipse.isInside(inside, radius * radius));
        
        assertFalse(ellipse.isInside(outside, -radius * radius));        
    }
    
    @Test
    public void testIsLocus() {
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

        Ellipse ellipse = new Ellipse(center, radius, radius, 0.0);
        
        //center is not locus
        assertFalse(ellipse.isLocus(center));
        assertFalse(ellipse.isLocus(center, ABSOLUTE_ERROR));

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
        assertFalse(ellipse.isLocus(inside));
        assertFalse(ellipse.isLocus(inside, ABSOLUTE_ERROR));
        
        assertFalse(ellipse.isLocus(outside));
        assertFalse(ellipse.isLocus(outside, ABSOLUTE_ERROR));
        
        
        //for point at locus of circle, distance is zero
        
        //zero is locus
        assertTrue(ellipse.isLocus(zero, ABSOLUTE_ERROR));
        assertTrue(ellipse.isLocus(zero, radius));
        
        //Force IllegalArgumentExcepetion
        try{
            ellipse.isLocus(zero, -radius);
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

            Ellipse ellipse = new Ellipse(center, radius, radius, 0.0);

            Point2D point = new HomogeneousPoint2D(
                    center.getInhomX() + radius * Math.cos(theta),
                    center.getInhomY() + radius * Math.sin(theta), 1.0); 
            point.normalize();

            assertTrue(ellipse.isLocus(point, ABSOLUTE_ERROR));

            //find tangent line at locus point
            Line2D line = ellipse.getTangentLineAt(point, ABSOLUTE_ERROR);

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
}
