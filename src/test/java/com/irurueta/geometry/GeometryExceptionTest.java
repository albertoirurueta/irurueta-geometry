/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.GeometryException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 1, 2012
 */
package com.irurueta.geometry;

import org.junit.*;
import static org.junit.Assert.assertNotNull;

public class GeometryExceptionTest {
    
    public GeometryExceptionTest() {
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
        GeometryException ex;
        assertNotNull(ex = new GeometryException());
        
        ex = null;
        assertNotNull(ex = new GeometryException("message"));
        
        ex = null;
        assertNotNull(ex = new GeometryException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new GeometryException("message", new Exception()));
    }
}
