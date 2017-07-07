/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.RefinerException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 28, 2017.
 */
package com.irurueta.geometry.refiners;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class RefinerExceptionTest {
    
    public RefinerExceptionTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testConstructor() {
        RefinerException ex;
        assertNotNull(ex = new RefinerException());
        
        ex = null;
        assertNotNull(ex = new RefinerException("message"));
        
        ex = null;
        assertNotNull(ex = new RefinerException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new RefinerException("message", new Exception()));
    }
}
