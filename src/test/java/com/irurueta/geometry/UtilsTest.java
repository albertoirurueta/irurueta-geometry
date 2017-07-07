/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.irurueta.geometry;

import com.irurueta.statistics.UniformRandomizer;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import static org.junit.Assert.*;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

public class UtilsTest {
    
    public static final double MIN_RADIANS = -Math.PI / 2.0;
    public static final double MAX_RADIANS = Math.PI / 2.0;
    
    public static final double MIN_DEGREES = -180.0;
    public static final double MAX_DEGREES = 180.0;
    
    public static final int MIN_TURNS = 500;
    public static final int MAX_TURNS = 10000;
    
    public static final double ABSOLUTE_ERROR = 1e-8;
    public static final double LARGE_ABSOLUTE_ERROR = 1e-6;
    
    public static final int TIMES = 50;
    
    public UtilsTest() {
    }
    
    @BeforeClass
    public static void setUpClass() {
    }
    
    @AfterClass
    public static void tearDownClass() {
    }
    
    @Before
    public void setUp() {
    }
    
    @After
    public void tearDown() {
    }
    
    @Test
    public void testConvertToDegrees(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double radians = randomizer.nextDouble(MIN_RADIANS, MAX_RADIANS);
        double degrees = radians * 180.0 / Math.PI;
        
        assertEquals(Utils.convertToDegrees(radians), degrees, ABSOLUTE_ERROR);
    }
    
    @Test
    public void testConvertToRadians(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double degrees = randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES);
        double radians = degrees * Math.PI / 180.0;
        
        assertEquals(Utils.convertToRadians(degrees), radians, ABSOLUTE_ERROR);
    }
    
    @Test
    public void testNormalizeAngleDegrees(){
        for(int t = 0; t < TIMES; t++){
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double degrees = randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES);
            int turns = (randomizer.nextBoolean() ? 1 : -1) * 
                    randomizer.nextInt(MIN_TURNS, MAX_TURNS);
            double degrees2 = degrees + 
                     turns * 360.0;

            double degrees3 = Utils.normalizeAngleDegrees(degrees2);
            double degrees4 = Utils.normalizeAngleDegrees(degrees);

            //check correctness
            assertEquals(degrees, degrees3, ABSOLUTE_ERROR);
            assertEquals(degrees, degrees4, ABSOLUTE_ERROR);   

            double degrees5 = degrees2;
            while(degrees5 <= -180.0){
                degrees5 += 2.0*180.0;
            }
            while(degrees5 > 180.0){
                degrees5 -= 2.0*180.0;
            }

            assertEquals(degrees, degrees5, LARGE_ABSOLUTE_ERROR);   
        }
    }
    
    @Test
    public void testNormalizeAngleRadians(){
        for(int t = 0; t < TIMES; t++){
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double radians = randomizer.nextDouble(MIN_RADIANS, MAX_RADIANS);
            int turns = (randomizer.nextBoolean() ? 1 : -1) * 
                    randomizer.nextInt(MIN_TURNS, MAX_TURNS);
            double radians2 = radians + 
                     turns * 2.0 * Math.PI;

            double radians3 = Utils.normalizeAngleRadians(radians2);
            double radians4 = Utils.normalizeAngleRadians(radians);


            //check correctness
            assertEquals(radians, radians3, ABSOLUTE_ERROR);
            assertEquals(radians, radians4, ABSOLUTE_ERROR);   
            
            double radians5 = radians2;
            while(radians5 <= -Math.PI){
                radians5 += 2.0*Math.PI;
            }
            while(radians5 > Math.PI){
                radians5 -= 2.0*Math.PI;
            }

            assertEquals(radians, radians5, LARGE_ABSOLUTE_ERROR);   
        }
    }    
}
