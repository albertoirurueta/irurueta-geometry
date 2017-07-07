/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.ConicNotAvailableException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 9, 2012
 */
package com.irurueta.geometry;

import org.junit.*;
import static org.junit.Assert.*;

public class ConicNotAvailableExceptionTest {
    
    public ConicNotAvailableExceptionTest() {
    }

    @BeforeClass
    public static void setUpClass() throws Exception {
    }

    @AfterClass
    public static void tearDownClass() throws Exception {
    }
    
    @Before
    public void setUp() {
    }
    
    @After
    public void tearDown() {
    }
    
    @Test
    public void testConstructor(){
        ConicNotAvailableException ex;
        assertNotNull(ex = new ConicNotAvailableException());
        
        ex = null;
        assertNotNull(ex = new ConicNotAvailableException("message"));
        
        ex = null;
        assertNotNull(ex = new ConicNotAvailableException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new ConicNotAvailableException("message", 
                new Exception()));
    }
}
