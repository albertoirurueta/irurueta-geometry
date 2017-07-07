/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.calib3d.Pattern2D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 1, 2015
 */
package com.irurueta.geometry.calib3d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class Pattern2DTest {
    
    public Pattern2DTest() {}
    
    @BeforeClass
    public static void setUpClass() {}
    
    @AfterClass
    public static void tearDownClass() {}
    
    @Before
    public void setUp() {}
    
    @After
    public void tearDown() {}

    @Test
    public void testCreate(){
        Pattern2D pattern = Pattern2D.create(Pattern2DType.QR);
        
        assertEquals(pattern.getType(), Pattern2DType.QR);
        assertTrue(pattern instanceof QRPattern2D);
        
        pattern = Pattern2D.create(Pattern2DType.CIRCLES);
        
        assertEquals(pattern.getType(), Pattern2DType.CIRCLES);
        assertTrue(pattern instanceof CirclesPattern2D);
    }
}
