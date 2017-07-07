/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.NonSymmetricMatrixException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 1, 2012
 */
package com.irurueta.geometry;

import static org.junit.Assert.assertNotNull;
import org.junit.*;

public class NonSymmetricMatrixExceptionTest {
    
    public NonSymmetricMatrixExceptionTest() {
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
        NonSymmetricMatrixException ex;
        assertNotNull(ex = new NonSymmetricMatrixException());
        
        ex = null;
        assertNotNull(ex = new NonSymmetricMatrixException("message"));
        
        ex = null;
        assertNotNull(ex = new NonSymmetricMatrixException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new NonSymmetricMatrixException("message", 
                new Exception()));
    }
}
