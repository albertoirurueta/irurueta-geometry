/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.ParallelVectorsException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 17, 2012
 */
package com.irurueta.geometry;

import static org.junit.Assert.assertNotNull;
import org.junit.*;

public class ParallelVectorsExceptionTest {
    
    public ParallelVectorsExceptionTest() {
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
        ParallelVectorsException ex;
        assertNotNull(ex = new ParallelVectorsException());
        
        ex = null;
        assertNotNull(ex = new ParallelVectorsException("message"));
        
        ex = null;
        assertNotNull(ex = new ParallelVectorsException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new ParallelVectorsException("message", 
                new Exception()));
    }        
}
