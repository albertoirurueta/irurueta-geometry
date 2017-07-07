/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.calib3d.CirclesPattern2D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 29, 2015
 */
package com.irurueta.geometry.calib3d;

import com.irurueta.geometry.Point2D;
import java.util.List;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class CirclesPattern2DTest {
    
    public CirclesPattern2DTest() {}
    
    @BeforeClass
    public static void setUpClass() {}
    
    @AfterClass
    public static void tearDownClass() {}
    
    @Before
    public void setUp() {}
    
    @After
    public void tearDown() {}

    @Test
    public void testConstructor(){
        CirclesPattern2D pattern = new CirclesPattern2D();
        
        //check default values
        assertEquals(pattern.getPointSeparation(), 
                CirclesPattern2D.DEFAULT_POINT_SEPARATION, 0.0);
        assertEquals(pattern.getCols(), CirclesPattern2D.DEFAULT_COLS);
        assertEquals(pattern.getRows(), CirclesPattern2D.DEFAULT_ROWS);
        assertEquals(pattern.getType(), Pattern2DType.CIRCLES);
        assertEquals(pattern.getNumberOfPoints(), 
                CirclesPattern2D.DEFAULT_NUMBER_OF_POINTS);
    }
    
    @Test
    public void testGetSetPointSeparation(){
        CirclesPattern2D pattern = new CirclesPattern2D();
        
        //check default value
        assertEquals(pattern.getPointSeparation(),
                CirclesPattern2D.DEFAULT_POINT_SEPARATION, 0.0);
        
        //set new value
        pattern.setPointSeparation(1.0);
        
        //check correctness
        assertEquals(pattern.getPointSeparation(), 1.0, 0.0);
        
        //force IllegalArgumentException
        try{
            pattern.setPointSeparation(0.0);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetCols(){
        CirclesPattern2D pattern = new CirclesPattern2D();
        
        //check default value
        assertEquals(pattern.getCols(), CirclesPattern2D.DEFAULT_COLS);
        assertEquals(pattern.getNumberOfPoints(), 
                CirclesPattern2D.DEFAULT_NUMBER_OF_POINTS);
        
        //set new value
        pattern.setCols(2);
        
        //check correctness
        assertEquals(pattern.getCols(), 2);
        assertEquals(pattern.getNumberOfPoints(), 
                pattern.getRows() * pattern.getCols());
        
        //force IllegalArgumentException
        try{
            pattern.setCols(1);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetRows(){
        CirclesPattern2D pattern = new CirclesPattern2D();
        
        //check default value
        assertEquals(pattern.getRows(), CirclesPattern2D.DEFAULT_ROWS);
        assertEquals(pattern.getNumberOfPoints(), 
                CirclesPattern2D.DEFAULT_NUMBER_OF_POINTS);        
        
        //set new value
        pattern.setRows(2);
        
        //check correctness
        assertEquals(pattern.getRows(), 2);
        assertEquals(pattern.getNumberOfPoints(), 
                pattern.getRows() * pattern.getCols());        
        
        //force IllegalArgumentException
        try{
            pattern.setRows(1);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetIdealPoints(){
        CirclesPattern2D pattern = new CirclesPattern2D();
        pattern.setPointSeparation(1.0);
        pattern.setCols(3);
        pattern.setRows(3);
        
        List<Point2D> points = pattern.getIdealPoints();
        
        assertEquals(points.get(0).getInhomX(), 0.0, 0.0);
        assertEquals(points.get(0).getInhomY(), 0.0, 0.0);
        
        assertEquals(points.get(1).getInhomX(), 2.0, 0.0);
        assertEquals(points.get(1).getInhomY(), 0.0, 0.0);
        
        assertEquals(points.get(2).getInhomX(), 4.0, 0.0);
        assertEquals(points.get(2).getInhomY(), 0.0, 0.0);
        
        assertEquals(points.get(3).getInhomX(), 1.0, 0.0);
        assertEquals(points.get(3).getInhomY(), 1.0, 0.0);
        
        assertEquals(points.get(4).getInhomX(), 3.0, 0.0);
        assertEquals(points.get(4).getInhomY(), 1.0, 0.0);
        
        assertEquals(points.get(5).getInhomX(), 5.0, 0.0);
        assertEquals(points.get(5).getInhomY(), 1.0, 0.0);
        
        assertEquals(points.get(6).getInhomX(), 0.0, 0.0);
        assertEquals(points.get(6).getInhomY(), 2.0, 0.0);
        
        assertEquals(points.get(7).getInhomX(), 2.0, 0.0);
        assertEquals(points.get(7).getInhomY(), 2.0, 0.0);
        
        assertEquals(points.get(8).getInhomX(), 4.0, 0.0);
        assertEquals(points.get(8).getInhomY(), 2.0, 0.0);
        
        assertEquals(points.size(), pattern.getNumberOfPoints());
    }
}
