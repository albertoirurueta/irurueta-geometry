/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.NormalizerException
 */
package com.irurueta.geometry.estimators;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 *
 * @author albertoirurueta
 */
public class NormalizerExceptionTest {
    
    public NormalizerExceptionTest() {
    }
    
    @BeforeClass
    public static void setUpClass() {
    }
    
    @AfterClass
    public static void tearDownClass() {
    }
    
    @Before
    public void setUp() {
    }
    
    @After
    public void tearDown() {
    }

    @Test
    public void testConstructor(){
        NormalizerException ex;
        assertNotNull(ex = new NormalizerException());
        
        ex = null;
        assertNotNull(ex = new NormalizerException("message"));
        
        ex = null;
        assertNotNull(ex = new NormalizerException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new NormalizerException("message", new Exception()));
    }
}
