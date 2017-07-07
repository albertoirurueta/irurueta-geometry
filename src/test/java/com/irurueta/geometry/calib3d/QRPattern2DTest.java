/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.calib3d.QRPattern2D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 1, 2015
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

public class QRPattern2DTest {
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    
    public QRPattern2DTest() {}
    
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
        QRPattern2D pattern = new QRPattern2D();
        
        //check default values
        assertEquals(pattern.getCodeWidth(),
                QRPattern2D.DEFAULT_QR_CODE_WIDTH, 0.0);
        assertEquals(pattern.getCodeHeight(),
                QRPattern2D.DEFAULT_QR_CODE_HEIGHT, 0.0);
        assertEquals(pattern.getType(), Pattern2DType.QR);
        assertEquals(pattern.getNumberOfPoints(), QRPattern2D.NUMBER_OF_POINTS);
    }
    
    @Test
    public void testGetSetCodeWidth(){
        QRPattern2D pattern = new QRPattern2D();
        
        //check default value
        assertEquals(pattern.getCodeWidth(),
                QRPattern2D.DEFAULT_QR_CODE_WIDTH, 0.0);
        
        //set new value
        pattern.setCodeWidth(5.0);
        
        //check correctness
        assertEquals(pattern.getCodeWidth(), 5.0, 0.0);
        
        //Force IllegalArgumentException
        try{
            pattern.setCodeWidth(0.0);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetSetCodeHeight(){
        QRPattern2D pattern = new QRPattern2D();
        
        //check default value
        assertEquals(pattern.getCodeHeight(),
                QRPattern2D.DEFAULT_QR_CODE_HEIGHT, 0.0);
        
        //set new value
        pattern.setCodeHeight(10.0);
        
        //check correctness
        assertEquals(pattern.getCodeHeight(), 10.0, 0.0);
        
        //Force IllegalArgumentException
        try{
            pattern.setCodeHeight(0.0);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testGetIdealPoints(){
        QRPattern2D pattern = new QRPattern2D();
        
        //check correctness
        List<Point2D> points = pattern.getIdealPoints();
        
        //check correctness
        assertEquals(points.size(), 4);
        
        //by default QR codes are assumed to be 1.1 x 1.1 centimeters
        
        //1st point is bottom-left finder pattern, which is located at:
        //(0cm, 0.792cm)
        assertEquals(points.get(0).getInhomX(), 0.0, 0.0);
        assertEquals(points.get(0).getInhomY(), 0.00792, ABSOLUTE_ERROR);
        
        //2nd point is top-left finder pattern, which is located at:
        //(0cm, 0cm)
        assertEquals(points.get(1).getInhomX(), 0.0, ABSOLUTE_ERROR);
        assertEquals(points.get(1).getInhomY(), 0.0, ABSOLUTE_ERROR);
        
        //3rd point is top-right finder pattern, which is located at:
        //(0.792cm, 0cm)
        assertEquals(points.get(2).getInhomX(), 0.00792, ABSOLUTE_ERROR);
        assertEquals(points.get(2).getInhomY(), 0.0, ABSOLUTE_ERROR);
        
        //4th point is bottom-right finder pattern, which is located at:
        //(0.66cm, 0.66cm)
        assertEquals(points.get(3).getInhomX(), 0.0066, ABSOLUTE_ERROR);
        assertEquals(points.get(3).getInhomY(), 0.0066, ABSOLUTE_ERROR);
        
        assertEquals(points.size(), pattern.getNumberOfPoints());
    }
}
