/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.CoincidentPlanesException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 9, 2012
 */
package com.irurueta.geometry;

import org.junit.*;
import static org.junit.Assert.*;

public class CoincidentPlanesExceptionTest {
    
    public CoincidentPlanesExceptionTest() {
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
        CoincidentPlanesException ex;
        assertNotNull(ex = new CoincidentPlanesException());
        
        ex = null;
        assertNotNull(ex = new CoincidentPlanesException("message"));
        
        ex = null;
        assertNotNull(ex = new CoincidentPlanesException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new CoincidentPlanesException("message", 
                new Exception()));
    }    
}
