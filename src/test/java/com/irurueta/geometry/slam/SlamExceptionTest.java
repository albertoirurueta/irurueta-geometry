/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.slam.SlamException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 6, 2016.
 */
package com.irurueta.geometry.slam;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class SlamExceptionTest {
    
    public SlamExceptionTest() {}
    
    @BeforeClass
    public static void setUpClass() {}
    
    @AfterClass
    public static void tearDownClass() {}
    
    @Before
    public void setUp() {}
    
    @After
    public void tearDown() {}

    @Test
    public void testConstructor() {
        SlamException ex;
        assertNotNull(ex = new SlamException());
        
        ex = null;
        assertNotNull(ex = new SlamException("message"));
        
        ex = null;
        assertNotNull(ex = new SlamException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new SlamException("message", new Exception()));
    }
}
