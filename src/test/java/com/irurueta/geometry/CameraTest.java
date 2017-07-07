/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.irurueta.geometry;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class CameraTest {
    
    public CameraTest() {
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
    public void testCreate(){
        Camera c = Camera.create();
        assertEquals(c.getType(), Camera.DEFAULT_CAMERA_TYPE);
        
        c = Camera.create(CameraType.PINHOLE_CAMERA);
        assertEquals(c.getType(), CameraType.PINHOLE_CAMERA);        
    }
}
