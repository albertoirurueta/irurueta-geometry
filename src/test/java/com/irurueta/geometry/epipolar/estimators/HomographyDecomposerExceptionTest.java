/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.epipolar.estimators.HomographyDecomposerException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date May 24, 2017.
 */
package com.irurueta.geometry.epipolar.estimators;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class HomographyDecomposerExceptionTest {
    
    public HomographyDecomposerExceptionTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testConstructor() {
        HomographyDecomposerException ex;
        assertNotNull(ex = new HomographyDecomposerException());
        
        ex = null;
        assertNotNull(ex = new HomographyDecomposerException("message"));
        
        ex = null;
        assertNotNull(ex = new HomographyDecomposerException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new HomographyDecomposerException("message", 
                new Exception()));
    }
}
