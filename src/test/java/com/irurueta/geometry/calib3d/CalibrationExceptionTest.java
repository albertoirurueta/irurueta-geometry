/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.calib3d.CalibrationException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 6, 2015
 */
package com.irurueta.geometry.calib3d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class CalibrationExceptionTest {
    
    public CalibrationExceptionTest() {}
    
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
        CalibrationException ex;
        assertNotNull(ex = new CalibrationException());
        
        ex = null;
        assertNotNull(ex = new CalibrationException("message"));
        
        ex = null;
        assertNotNull(ex = new CalibrationException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new CalibrationException("message", 
                new Exception()));
    }
}
