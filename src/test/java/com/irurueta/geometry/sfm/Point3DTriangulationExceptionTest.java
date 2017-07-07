/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.sfm.Point3DTriangulationException
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

public class Point3DTriangulationExceptionTest {
    
    public Point3DTriangulationExceptionTest() {}
    
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
        Point3DTriangulationException ex;
        assertNotNull(ex = new Point3DTriangulationException());
        
        ex = null;
        assertNotNull(ex = new Point3DTriangulationException("message"));
        
        ex = null;
        assertNotNull(ex = new Point3DTriangulationException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new Point3DTriangulationException("message", 
                new Exception()));
    }
}
