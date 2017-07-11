/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.slam.AbsoluteOrientationConstantVelocityModelSlamCalibrationData
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date October 12, 2016.
 */
package com.irurueta.geometry.slam;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.InvalidCovarianceMatrixException;
import com.irurueta.statistics.MultivariateNormalDist;
import com.irurueta.statistics.UniformRandomizer;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class AbsoluteOrientationConstantVelocityModelSlamCalibrationDataTest {
    
    public static final double ABSOLUTE_ERROR = 1e-8;
    
    public AbsoluteOrientationConstantVelocityModelSlamCalibrationDataTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testConstructorGetControlLengthAndGetStateLength() {
        AbsoluteOrientationConstantVelocityModelSlamCalibrationData data = 
                new AbsoluteOrientationConstantVelocityModelSlamCalibrationData();
        
        //check initial values
        assertEquals(data.getControlLength(), 
                AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH);
        assertEquals(data.getStateLength(), 
                AbsoluteOrientationConstantVelocityModelSlamEstimator.STATE_LENGTH);
        assertNull(data.getControlMean());
        assertNull(data.getControlCovariance());
    }
    
    @Test
    public void testGetSetControlMean() {
        AbsoluteOrientationConstantVelocityModelSlamCalibrationData data = 
                new AbsoluteOrientationConstantVelocityModelSlamCalibrationData();
        
        //check initial value
        assertNull(data.getControlMean());
        
        //set new value
        double[] mean = new double[
                AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH];
        data.setControlMean(mean);
        
        //check correctness
        assertSame(data.getControlMean(), mean);
        
        //Force IllegalArgumentException
        double[] wrong = new double[1];
        try {
            data.setControlMean(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testGetSetControlCovariance() throws WrongSizeException {
        AbsoluteOrientationConstantVelocityModelSlamCalibrationData data = 
                new AbsoluteOrientationConstantVelocityModelSlamCalibrationData();
        
        //check initial value
        assertNull(data.getControlCovariance());
        
        //set new value
        Matrix cov = new Matrix(
                AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH,
                AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH);
        data.setControlCovariance(cov);
        
        //check correctness
        assertSame(data.getControlCovariance(), cov);
        
        //Force IllegalArgumentException
        Matrix wrong = new Matrix(1,1);
        try {
            data.setControlCovariance(wrong);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testSetControlMeanAndCovariance() throws WrongSizeException {
        AbsoluteOrientationConstantVelocityModelSlamCalibrationData data = 
                new AbsoluteOrientationConstantVelocityModelSlamCalibrationData();
        
        //set new values
        double[] mean = new double[
                AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH];
        Matrix cov = new Matrix(
                AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH,
                AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH);  
        data.setControlMeanAndCovariance(mean, cov);
        
        //check correctness
        assertSame(data.getControlMean(), mean);
        assertSame(data.getControlCovariance(), cov);
        
        //Force IllegalArgumentException
        double[] wrongMean = new double[1];
        Matrix wrongCov = new Matrix(1,1);
        
        try {
            data.setControlMeanAndCovariance(wrongMean, cov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            data.setControlMeanAndCovariance(mean, wrongCov);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    

    @Test
    public void testPropagateWithControlJacobian() throws WrongSizeException, 
            InvalidCovarianceMatrixException {
        
        Matrix cov = Matrix.identity(
                AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH, 
                AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH).
                multiplyByScalarAndReturnNew(1e-3);
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] mean = new double[
                AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH];
        randomizer.fill(mean);
        
        AbsoluteOrientationConstantVelocityModelSlamCalibrationData data = 
                new AbsoluteOrientationConstantVelocityModelSlamCalibrationData();
        data.setControlMeanAndCovariance(mean, cov);
        
        Matrix jacobian = Matrix.identity(
                AbsoluteOrientationConstantVelocityModelSlamEstimator.STATE_LENGTH, 
                AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH).
                multiplyByScalarAndReturnNew(2.0);
        
        MultivariateNormalDist dist = data.propagateWithControlJacobian(
                jacobian);
        MultivariateNormalDist dist2 = new MultivariateNormalDist();
        data.propagateWithControlJacobian(jacobian, dist2);
        
        //check correctness
        Matrix propagatedCov = jacobian.multiplyAndReturnNew(cov).
                multiplyAndReturnNew(jacobian.transposeAndReturnNew());
        
        assertTrue(dist.getCovariance().equals(propagatedCov, ABSOLUTE_ERROR));
        assertTrue(dist2.getCovariance().equals(propagatedCov, ABSOLUTE_ERROR));
        
        assertArrayEquals(dist.getMean(), 
                new double[AbsoluteOrientationConstantVelocityModelSlamEstimator.STATE_LENGTH], 
                0.0);
        assertArrayEquals(dist2.getMean(), 
                new double[AbsoluteOrientationConstantVelocityModelSlamEstimator.STATE_LENGTH], 
                0.0);        
    }}