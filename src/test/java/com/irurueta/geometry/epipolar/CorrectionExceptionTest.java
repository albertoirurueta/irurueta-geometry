/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.epipolar.CorrectionException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 27, 2015
 */
package com.irurueta.geometry.epipolar;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class CorrectionExceptionTest {
    
    public CorrectionExceptionTest() {}
    
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
        CorrectionException ex;
        assertNotNull(ex = new CorrectionException());
        
        ex = null;
        assertNotNull(ex = new CorrectionException("message"));
        
        ex = null;
        assertNotNull(ex = new CorrectionException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new CorrectionException("message", new Exception()));
    }
}
