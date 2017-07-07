/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.AffineParameters2D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date October 28, 2012
 */
package com.irurueta.geometry;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import static org.junit.Assert.*;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

public class AffineParameters2DTest {
    
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final int INHOM_COORDS = 2;
    
    public static final double ABSOLUTE_ERROR = 1e-8;
    
    public AffineParameters2DTest() {
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
    public void testConstructors() throws WrongSizeException{
        //Test empty constructor
        AffineParameters2D params = new AffineParameters2D();
        
        //check correctness
        assertEquals(params.getScaleX(), AffineParameters2D.DEFAULT_SCALE, 0.0);
        assertEquals(params.getScaleY(), AffineParameters2D.DEFAULT_SCALE, 0.0);
        assertEquals(params.getSkewness(), AffineParameters2D.DEFAULT_SKEWNESS, 
                0.0);
        
        //Test cosntructor with scale
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        params = new AffineParameters2D(scale);
        
        //check correctness
        assertEquals(params.getScaleX(), scale, 0.0);
        assertEquals(params.getScaleY(), scale, 0.0);
        assertEquals(params.getSkewness(), AffineParameters2D.DEFAULT_SKEWNESS, 
                0.0);
        
        //Test constructor with scale and skewness
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        params = new AffineParameters2D(scale, skewness);
        
        //check correctness
        assertEquals(params.getScaleX(), scale, 0.0);
        assertEquals(params.getScaleY(), scale, 0.0);
        assertEquals(params.getSkewness(), skewness, 0.0);
        
        //Test constructor with scaleX, scaleY and skewness
        double scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        params = new AffineParameters2D(scaleX, scaleY, skewness);
        
        //check correctness
        assertEquals(params.getScaleX(), scaleX, 0.0);
        assertEquals(params.getScaleY(), scaleY, 0.0);
        assertEquals(params.getSkewness(), skewness, 0.0);
        
        //Test constructor with matrix
        Matrix m = new Matrix(INHOM_COORDS, INHOM_COORDS);
        m.setElementAt(0, 0, scaleX);
        m.setElementAt(1, 1, scaleY);
        m.setElementAt(0, 1, skewness);
        
        params = new AffineParameters2D(m);
        
        //check correctness
        assertEquals(params.getScaleX(), scaleX, 0.0);
        assertEquals(params.getScaleY(), scaleY, 0.0);
        assertEquals(params.getSkewness(), skewness, 0.0);
        
        //Force IllegalArgumentException (invalid size)
        Matrix badM1 = new Matrix(INHOM_COORDS + 1, INHOM_COORDS + 1);
        params = null;
        try{
            params = new AffineParameters2D(badM1);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        //Force IllegalArgumentException (non upper triangular matrix
        Matrix badM2 = new Matrix(INHOM_COORDS, INHOM_COORDS);
        badM2.setElementAt(1, 0, 1.0);
        try{
            params = new AffineParameters2D(badM2);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(params);
        
        //Test constructor with matrix and threshold
        double threshold = randomizer.nextDouble();
        params = new AffineParameters2D(m, threshold);
        
        //check correctness
        assertEquals(params.getScaleX(), scaleX, 0.0);
        assertEquals(params.getScaleY(), scaleY, 0.0);
        assertEquals(params.getSkewness(), skewness, 0.0);
        
        //Force IllegalArgumentException (invalid size)
        params = null;
        try{
            params = new AffineParameters2D(badM1, threshold);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        //Force IllegalArgumentException (non upper triangular matrix)
        badM2.setElementAt(1, 0, threshold + 1.0);
        try{
            params = new AffineParameters2D(badM2);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        //Force IllegalArgumentException (with negative threshold)
        try{
            params = new AffineParameters2D(m, -threshold);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(params);
    }
    
    @Test
    public void testGetSetScaleX(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        AffineParameters2D params = new AffineParameters2D();
        
        //check default values
        assertEquals(params.getScaleX(), AffineParameters2D.DEFAULT_SCALE, 0.0);
        
        //set new value
        params.setScaleX(scaleX);
        
        //check correctness
        assertEquals(params.getScaleX(), scaleX, 0.0);
    }
    
    @Test
    public void testGetSetScaleY(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        AffineParameters2D params = new AffineParameters2D();
        
        //check default values
        assertEquals(params.getScaleY(), AffineParameters2D.DEFAULT_SCALE, 0.0);
        
        //set new value
        params.setScaleY(scaleY);
        
        //check correctness
        assertEquals(params.getScaleY(), scaleY, 0.0);
    }
    
    @Test
    public void testSetScale(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        AffineParameters2D params = new AffineParameters2D();
        
        //check default values
        assertEquals(params.getScaleX(), AffineParameters2D.DEFAULT_SCALE, 0.0);
        assertEquals(params.getScaleY(), AffineParameters2D.DEFAULT_SCALE, 0.0);
        
        //set new value
        params.setScale(scale);
        
        //check correctness
        assertEquals(params.getScaleX(), scale, 0.0);
        assertEquals(params.getScaleY(), scale, 0.0);
    }
    
    @Test
    public void testGetSetSkewness(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        AffineParameters2D params = new AffineParameters2D();
        
        //check default values
        assertEquals(params.getSkewness(), AffineParameters2D.DEFAULT_SKEWNESS, 
                0.0);
        
        //set new value
        params.setSkewness(skewness);
        
        //check correctness
        assertEquals(params.getSkewness(), skewness, 0.0);
    }
    
    @Test
    public void testAsMatrix() throws WrongSizeException{
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        AffineParameters2D params = new AffineParameters2D(scaleX, scaleY,
                skewness);
                
        Matrix expectedMatrix = new Matrix(INHOM_COORDS, INHOM_COORDS);
        expectedMatrix.initialize(0.0);
        expectedMatrix.setElementAt(0, 0, scaleX);
        expectedMatrix.setElementAt(1, 1, scaleY);
        expectedMatrix.setElementAt(0, 1, skewness);        
        
        Matrix m1 = params.asMatrix();
        Matrix m2 = new Matrix(INHOM_COORDS, INHOM_COORDS);
        params.asMatrix(m2);
        
        assertTrue(expectedMatrix.equals(m1, ABSOLUTE_ERROR));
        assertTrue(expectedMatrix.equals(m2, ABSOLUTE_ERROR));
    }
    
    @Test
    public void testFromMatrixAndIsValidMatrix() throws WrongSizeException{
        UniformRandomizer randomizer = new UniformRandomizer(new Random());        
        double scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double threshold = randomizer.nextDouble();
        
        //build valid matrix
        Matrix m = new Matrix(INHOM_COORDS, INHOM_COORDS);
        m.setElementAt(0, 0, scaleX);
        m.setElementAt(1, 1, scaleY);
        m.setElementAt(0, 1, skewness);
        
        //check matrix is valid
        assertTrue(AffineParameters2D.isValidMatrix(m));
        assertTrue(AffineParameters2D.isValidMatrix(m, threshold));
        
        //Force IllegalArgumentException when checking validity (negative 
        //threshold)
        try{
            AffineParameters2D.isValidMatrix(m, -threshold);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
                
        AffineParameters2D params = new AffineParameters2D();        
        
        //set parameters from matrix
        params.fromMatrix(m);
        
        //check correctness
        assertEquals(params.getScaleX(), scaleX, 0.0);
        assertEquals(params.getScaleY(), scaleY, 0.0);
        assertEquals(params.getSkewness(), skewness, 0.0);
        
        //Force IllegalArgumentException (invalid size)
        Matrix badM1 = new Matrix(INHOM_COORDS + 1, INHOM_COORDS + 1);
        try{
            params.fromMatrix(badM1);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        //Force IllegalArgumentException (non upper triangular matrix
        Matrix badM2 = new Matrix(INHOM_COORDS, INHOM_COORDS);
        badM2.setElementAt(1, 0, 1.0);
        try{
            params.fromMatrix(badM2);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        //Test from matrix with threshold
        params.fromMatrix(m, threshold);
        
        //check correctness
        assertEquals(params.getScaleX(), scaleX, 0.0);
        assertEquals(params.getScaleY(), scaleY, 0.0);
        assertEquals(params.getSkewness(), skewness, 0.0);
        
        //Force IllegalArgumentException (invalid size)
        try{
            params.fromMatrix(badM1, threshold);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        //Force IllegalArgumentException (non upper triangular matrix
        badM2.setElementAt(1, 0, threshold + 1.0);
        try{
            params.fromMatrix(badM2, threshold);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        //Force IllegalArgumentException (with negative threshold)
        try{
            params.fromMatrix(m, -threshold);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
    }
}
