/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.DualConicNotAvailableException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 9, 2012
 */
package com.irurueta.geometry;

import static org.junit.Assert.assertNotNull;
import org.junit.*;

public class DualConicNotAvailableExceptionTest {
    
    public DualConicNotAvailableExceptionTest() {
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
        DualConicNotAvailableException ex;
        assertNotNull(ex = new DualConicNotAvailableException());
        
        ex = null;
        assertNotNull(ex = new DualConicNotAvailableException("message"));
        
        ex = null;
        assertNotNull(ex = new DualConicNotAvailableException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new DualConicNotAvailableException("message", 
                new Exception()));
    }
}
