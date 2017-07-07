/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.epipolar.estimators.HomographyDecomposition
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date May 24, 2017.
 */
package com.irurueta.geometry.epipolar.estimators;

import com.irurueta.geometry.EuclideanTransformation3D;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class HomographyDecompositionTest {
    
    public HomographyDecompositionTest() { }
    
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
        //empty constructor
        HomographyDecomposition decomposition = new HomographyDecomposition();
        
        //check default values
        assertNull(decomposition.getTransformation());
        assertNull(decomposition.getPlaneNormal());
        assertEquals(decomposition.getPlaneDistance(), 0.0, 0.0);
        
        //test non-empty constructor
        EuclideanTransformation3D transformation = 
                new EuclideanTransformation3D();
        double[] planeNormal = new double[
                HomographyDecomposition.PLANE_NORMAL_LENGTH];
        decomposition = new HomographyDecomposition(transformation, 
                planeNormal, 5.0);
        
        //check correctness
        assertSame(decomposition.getTransformation(), transformation);
        assertSame(decomposition.getPlaneNormal(), planeNormal);
        assertEquals(decomposition.getPlaneDistance(), 5.0, 0.0);
        
        //Force IllegalArgumentException
        decomposition = null;
        double[] wrong = new double[1];
        try {
            decomposition = new HomographyDecomposition(transformation, wrong, 
                    3.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(decomposition);
    }
    
    @Test
    public void testGetSetTransformation() {
        HomographyDecomposition decomposition = new HomographyDecomposition();
        
        //check default value
        assertNull(decomposition.getTransformation());
        
        //set new value
        EuclideanTransformation3D transformation =
                new EuclideanTransformation3D();
        decomposition.setTransformation(transformation);
        
        //check correctness
        assertSame(decomposition.getTransformation(), transformation);
    }
    
    @Test
    public void testGetSetPlaneNormal() {
        HomographyDecomposition decomposition = new HomographyDecomposition();
        
        //check default value
        assertNull(decomposition.getPlaneNormal());
        
        //set new value
        double[] planeNormal = new double[
                HomographyDecomposition.PLANE_NORMAL_LENGTH];
        decomposition.setPlaneNormal(planeNormal);
        
        //check correctness
        assertSame(decomposition.getPlaneNormal(), planeNormal);
        
        //force IllegalArgumentException
        double[] wrong = new double[1];
        try {
            decomposition.setPlaneNormal(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testGetSetPlaneDistance() {
        HomographyDecomposition decomposition = new HomographyDecomposition();
        
        //initial value
        assertEquals(decomposition.getPlaneDistance(), 0.0, 0.0);
        
        //set new value
        decomposition.setPlaneDistance(10.0);
        
        //check correctness
        assertEquals(decomposition.getPlaneDistance(), 10.0, 0.0);
    }
}
