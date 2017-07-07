/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.QuadricNotAvailableException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date August 14, 2012
 */
package com.irurueta.geometry;

import static org.junit.Assert.assertNotNull;
import org.junit.*;

public class QuadricNotAvailableExceptionTest {
    
    public QuadricNotAvailableExceptionTest() {
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
        QuadricNotAvailableException ex;
        assertNotNull(ex = new QuadricNotAvailableException());
        
        ex = null;
        assertNotNull(ex = new QuadricNotAvailableException("message"));
        
        ex = null;
        assertNotNull(ex = new QuadricNotAvailableException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new QuadricNotAvailableException("message",
                new Exception()));
    }
}
