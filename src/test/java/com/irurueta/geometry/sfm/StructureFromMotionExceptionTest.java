/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.sfm.StructureFromMotionException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 29, 2015
 */
package com.irurueta.geometry.sfm;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class StructureFromMotionExceptionTest {
    
    public StructureFromMotionExceptionTest() { }
    
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
        StructureFromMotionException ex;
        assertNotNull(ex = new StructureFromMotionException());
        
        ex = null;
        assertNotNull(ex = new StructureFromMotionException("message"));
        
        ex = null;
        assertNotNull(ex = new StructureFromMotionException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new StructureFromMotionException("message", 
                new Exception()));
    }
}
