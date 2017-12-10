/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.sfm.Sample2D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date December 28, 2016.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.Point2D;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class Sample2DTest {
    
    public Sample2DTest() { }
    
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
        Sample2D s = new Sample2D();
        
        //check default values
        assertNull(s.getId());
        assertEquals(s.getViewId(), 0);
        assertNull(s.getPoint());
        assertNull(s.getReconstructedPoint());
        assertEquals(s.getQualityScore(), Sample2D.DEFAULT_QUALITY_SCORE, 0.0);
        assertNull(s.getCovariance());
        assertNull(s.getColorData());
    }
    
    @Test
    public void testGetSetId() {
        Sample2D s = new Sample2D();
        
        //check default value
        assertNull(s.getId());
        
        //set new value
        s.setId("id");
        
        //check correctness
        assertEquals(s.getId(), "id");
    }
    
    @Test
    public void testGetSetViewId() {
        Sample2D s = new Sample2D();
        
        //check default values
        assertEquals(s.getViewId(), 0);
        
        //set new value
        s.setViewId(5);
        
        //check correctness
        assertEquals(s.getViewId(), 5);
    }
    
    @Test
    public void testGetSetPoint() {
        Sample2D s = new Sample2D();
        
        //check default value
        assertNull(s.getPoint());
        
        //set new value
        Point2D p = Point2D.create();
        s.setPoint(p);
        
        //check correctness
        assertSame(s.getPoint(), p);
    }

    @Test
    public void testGetSetReconstructedPoint() {
        Sample2D s = new Sample2D();

        //check default value
        assertNull(s.getReconstructedPoint());

        //set new value
        ReconstructedPoint3D reconstructedPoint3D = new ReconstructedPoint3D();
        s.setReconstructedPoint(reconstructedPoint3D);

        //check correctness
        assertSame(s.getReconstructedPoint(), reconstructedPoint3D);
    }
    
    @Test
    public void testGetSetQualityScore() {
        Sample2D s = new Sample2D();
        
        //check default value
        assertEquals(s.getQualityScore(), Sample2D.DEFAULT_QUALITY_SCORE, 0.0);
        
        //set new value
        s.setQualityScore(15.0);
        
        //check correctness
        assertEquals(s.getQualityScore(), 15.0, 0.0);
    }
    
    @Test
    public void testGetSetCovariance() throws WrongSizeException {
        Sample2D s = new Sample2D();
        
        //check default value
        assertNull(s.getCovariance());
        
        //set new value
        Matrix cov = new Matrix(2, 2);
        s.setCovariance(cov);
        
        //check correctness
        assertSame(s.getCovariance(), cov);
    }
    
    @Test
    public void testGetSetColorData() {
        Sample2D s = new Sample2D();
        
        //check default value
        assertNull(s.getColorData());
        
        //set new value
        PointColorData data = new CustomPointColorData();
        s.setColorData(data);
        
        //check correctness
        assertSame(s.getColorData(), data);
    }
    
    public static class CustomPointColorData extends PointColorData {

        @Override
        public void average(PointColorData other, PointColorData result) { }        
    }        
}
