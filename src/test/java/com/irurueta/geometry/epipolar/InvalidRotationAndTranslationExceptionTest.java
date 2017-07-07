/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.epipolar.InvalidRotationAndTranslationException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 25, 2015
 */
package com.irurueta.geometry.epipolar;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class InvalidRotationAndTranslationExceptionTest {
    
    public InvalidRotationAndTranslationExceptionTest() {}
    
    @BeforeClass
    public static void setUpClass() {}
    
    @AfterClass
    public static void tearDownClass() {}
    
    @Before
    public void setUp() {}
    
    @After
    public void tearDown() {}

    @Test
    public void testConstructor(){
        InvalidRotationAndTranslationException ex;
        assertNotNull(ex = new InvalidRotationAndTranslationException());
        
        ex = null;
        assertNotNull(ex = new InvalidRotationAndTranslationException(
                "message"));
        
        ex = null;
        assertNotNull(ex = new InvalidRotationAndTranslationException(
                new Exception()));
        
        ex = null;
        assertNotNull(ex = new InvalidRotationAndTranslationException("message",
                new Exception()));
    }
}
