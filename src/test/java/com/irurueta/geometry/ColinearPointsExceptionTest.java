/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.ColinearPointsException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 17, 2012
 */
package com.irurueta.geometry;

import static org.junit.Assert.assertNotNull;
import org.junit.*;

public class ColinearPointsExceptionTest {
    
    public ColinearPointsExceptionTest() {
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
        ColinearPointsException ex;
        assertNotNull(ex = new ColinearPointsException());
        
        ex = null;
        assertNotNull(ex = new ColinearPointsException("message"));
        
        ex = null;
        assertNotNull(ex = new ColinearPointsException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new ColinearPointsException("message", 
                new Exception()));
    }    
}
