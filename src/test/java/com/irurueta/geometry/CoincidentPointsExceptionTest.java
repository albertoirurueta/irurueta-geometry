/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.CoincidentLinesException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 9, 2012
 */
package com.irurueta.geometry;

import org.junit.*;
import static org.junit.Assert.*;

public class CoincidentPointsExceptionTest {
    
    public CoincidentPointsExceptionTest() {
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
        CoincidentPointsException ex;
        assertNotNull(ex = new CoincidentPointsException());
        
        ex = null;
        assertNotNull(ex = new CoincidentPointsException("message"));
        
        ex = null;
        assertNotNull(ex = new CoincidentPointsException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new CoincidentPointsException("message", 
                new Exception()));
    }
}
