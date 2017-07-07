/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.DualQuadricNotAvailableException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date August 13, 2012
 */
package com.irurueta.geometry;

import org.junit.*;
import static org.junit.Assert.*;

public class DualQuadricNotAvailableExceptionTest {
    
    public DualQuadricNotAvailableExceptionTest() {
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
        DualQuadricNotAvailableException ex;
        assertNotNull(ex = new DualQuadricNotAvailableException());
        
        ex = null;
        assertNotNull(ex = new DualQuadricNotAvailableException("message"));
        
        ex = null;
        assertNotNull(ex = new DualQuadricNotAvailableException("message",
                new Exception()));
    }
}
