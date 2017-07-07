/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.epipolar.EpipolarException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 12, 2015
 */
package com.irurueta.geometry.epipolar;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class EpipolarExceptionTest {
    
    public EpipolarExceptionTest() {}
    
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
        EpipolarException ex;
        assertNotNull(ex = new EpipolarException());
        
        ex = null;
        assertNotNull(ex = new EpipolarException("message"));
        
        ex = null;
        assertNotNull(ex = new EpipolarException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new EpipolarException("message", new Exception()));
    }
}
