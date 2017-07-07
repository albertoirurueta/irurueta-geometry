/**
 * @file
 * This file contains definition of
 * com.irurueta.geometry.Line2D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 3, 2012
 */
package com.irurueta.geometry;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import java.io.Serializable;

/**
 * Line2D in R2. Lines can be expressed using the following expression:
 * A * x + B * y + C = 0
 * where A and B are different from zero. Changing this expresion to a
 * y = m * x + b format leads to the following definitions:
 * m: slope of the line. Defined as m = -A/B
 * b: interception point of this line. Defined as b = -c/B
 * angle of the line with respect to the x axis = Math.atan(slope)
 */
public class Line2D implements Serializable{
    
    /**
     * Number of line parameters
     */
    public static final int LINE_NUMBER_PARAMS = 3;
    
    /**
     * Constant defining the size of vector that define the direction of a line
     */
    private static final int INHOM_VECTOR_SIZE = 2;
    
    
    /**
     * Positive threshold determine whether points lay inside (is locus) of a 
     * given line or not
     */
    public static final double DEFAULT_LOCUS_THRESHOLD = 1e-12;

    /**
     * Minimum allowed threshold
     */
    public static final double MIN_THRESHOLD = 0.0;
    
    /**
     * Defines the threshold used when comparing two values.
     */
    public static final double DEFAULT_COMPARISON_THRESHOLD = 1e-10;        
    
    /**
     * Machine precision
     */
    private static final double PRECISION = 1e-12;


    /**
     * Parameter A of a line
     */
    private double mA;
    
    /**
     * Parameter B of a line
     */
    private double mB;
    
    /**
     * Parameter C of a line
     */
    private double mC;
    
    /**
     * Indicates if line is normalized o rnot
     */
    private boolean mNormalized;
        
    /**
     * Constructor
     */
    public Line2D(){
        mA = mB = mC = 0.0;
        mNormalized = false;
    }
    
    /**
     * Constructor with parameters. Parameters of a line are provided in the
     * following homogeneous format:
     * A * x + B * y + C = 0
     * @param a Parameter A of a line
     * @param b Parameter B of a line
     * @param c Parameter C of a line
     */
    public Line2D(double a, double b, double c){
        setParameters(a, b, c);
    }
    
    /**
     * Constructor. This constructor takes two 2D points to build a line passing
     * through both of them.
     * @param pointA First point used to compute the line
     * @param pointB Second point used to compute the line
     * @param noThrow If true no exception is thrown even if points are 
     * coincident.
     * @throws CoincidentPointsException Raised if points are equal.
     */
    public Line2D(Point2D pointA, Point2D pointB, boolean noThrow)
        throws CoincidentPointsException{
        setParametersFromPairOfPoints(pointA, pointB, noThrow);
    }
    
    /**
     * Constructor. This constructor takes two 2D points to build a line passing
     * through both of them.
     * @param pointA First point used to compute the line
     * @param pointB Second point used to compute the line
     */    
    public Line2D(Point2D pointA, Point2D pointB){
        setParametersFromPairOfPoints(pointA, pointB);
    }
    
    /**
     * Constructor.
     * @param array Array contianing the three parameters of a line (A, B, C)
     * @throws IllegalArgumentException Raised if length of provided array is 
     * not three.
     */
    public Line2D(double[] array) throws IllegalArgumentException{
        setParameters(array);
    }
    
    /**
     * Constructor of a line from one point and its director vector
     * @param point point passing through the line
     * @param vector director vector
     * @throws IllegalArgumentException raised if vector length is not 2
     */
    public Line2D(Point2D point, double[] vector)
            throws IllegalArgumentException{
        setParametersFromPointAndDirectorVector(point, vector);
    }
    
    /**
     * Returns parameter A of this line
     * @return Parameter A of this line
     */
    public double getA(){
        return mA;
    }
    
    /**
     * Returns parameter B of this line
     * @return Parameter B of this line
     */
    public double getB(){
        return mB;
    }
    
    /**
     * Returns parameter C of this line
     * @return Parameter C of this line
     */
    public double getC(){
        return mC;
    }
    
    /**
     * Sets parameters of this line
     * @param a Parameter A of this line
     * @param b Parameter B of this line
     * @param c Parameter C of this line
     */
    public final void setParameters(double a, double b, double c){
        mA = a;
        mB = b;
        mC = c;
        mNormalized = false;
    }
    
    /**
     * Sets parameters of this line
     * @param array Array containing parameters of this line.
     * @throws IllegalArgumentException Raised if provided array does not
     * have length equal to 3
     */
    public final void setParameters(double[] array) 
            throws IllegalArgumentException{
        if(array.length != LINE_NUMBER_PARAMS) 
            throw new IllegalArgumentException();
        
        mA = array[0];
        mB = array[1];
        mC = array[2];
        mNormalized = false;
    }
    
    /**
     * Sets parameter A of this line
     * @param a Parameter A of this line
     */
    public void setA(double a){
        mA = a;
        mNormalized = false;
    }
    
    /**
     * Sets parameter B of this line
     * @param b Parameter B of this line
     */
    public void setB(double b){
        mB = b;
        mNormalized = false;
    }
    
    /**
     * Sets parameter C of this line
     * @param c Parameter C of this line
     */
    public void setC(double c){
        mC = c;
        mNormalized = false;
    }
    
    /**
     * Returns the slope of this line.
     * @return Slope of this line
     */
    public double getSlope(){
        return (-mA / mB);
    }
    
    /**
     * Sets the slope of this line
     * @param slope Slope of this line
     */
    public void setSlope(double slope){
        normalize();
        if(Math.abs(slope) > 1.0){
            mA = 1.0;
            mB = -mA / slope;
        }else{
            mB = 1.0;
            mA = -mB * slope;
        }
        mNormalized = false;
    }
    
    /**
     * Returns the angle of this line in radians
     * @return Angle of this line in radians.
     */
    public double getAngle(){
        double alpha = Math.atan2(-mA, mB);
        //if alpha is not between -90 and 90 degrees, then fix angle
        if(alpha > Math.PI / 2.0){            
            alpha = alpha - Math.PI;
        }else if(alpha < -Math.PI / 2.0){
            alpha = alpha + Math.PI;
        }
        
        return alpha;
    }
    
    /**
     * Sets angle of this line in radians
     * @param angle Angle of this line in radians
     */
    public void setAngle(double angle){
        double slope = Math.tan(angle);        
        setSlope(slope);
    }
    
    /**
     * Returns the y-coordinate intercept point of this line.
     * For a line following the expression A * X + B * y + C = 0 this method
     * evaluates y for x = 0
     * @return Vertical coordinate where the line intercepts with the Y axis
     */
    public double getYIntercept(){
        return -(mC / mB);
    }
    
    /**
     * Sets the y-coordinate intercept point of this line.
     * For a line following the expression A * x + B * y + C = 0 this method
     * recalculates the parameters that define the line
     * @param yIntercept Vertical coordinate where the line intercepts with the
     * Y axis.
     */
    public void setYIntercept(double yIntercept){
        mC = -mB * yIntercept;
        mNormalized = false;
    }
    
    /**
     * Sets parameters of this line from a pair of 2D points. The line will pass
     * through both points
     * @param pointA First point laying on this line
     * @param pointB Second point laying on this line
     * @param noThrow Enables/disables throwing exceptions
     * @throws CoincidentPointsException Raised if provided points are equal.
     */
    public final void setParametersFromPairOfPoints(Point2D pointA, 
            Point2D pointB, boolean noThrow) throws CoincidentPointsException{
        
        try{
            pointA.normalize();
            pointB.normalize();
        
            Matrix m = new Matrix(2, 
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH);
        
            m.setElementAt(0, 0, pointA.getHomX());
            m.setElementAt(0, 1, pointA.getHomY());
            m.setElementAt(0, 2, pointA.getHomW());
        
            m.setElementAt(1, 0, pointB.getHomX());
            m.setElementAt(1, 1, pointB.getHomY());
            m.setElementAt(1, 2, pointB.getHomW());
        
            //m = U * S * V', where m is 2x3, U is 2x3, S is 3x3 and V is 3x3
            SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();
        
            //because we have only 2 points, matrix will be rank 2 only if 
            //points are not linearly dependent, if they are, then they are 
            //coincident because homogeneous points are defined up to scale
            if(decomposer.getRank() < 2 && !noThrow)
                throw new CoincidentPointsException();
        
            //Because m is rank 2, the last column of V, from now on l, will be 
            //the right null-space of m. Hence m*l = 0, which is the equation of
            //a line.
            //Hence l is the null space for both pointA and pointB, or in other 
            //words, pointA and pointB are the locus of l, or l is the line 
            //passing through both pointA and pointB
            Matrix V = decomposer.getV();
        
            mA = V.getElementAt(0, 2);
            mB = V.getElementAt(1, 2);
            mC = V.getElementAt(2, 2);
            mNormalized = false;
        }catch(AlgebraException ignore){}
    }
    
    /**
     * Sets parameters of this line from a pair of 2D points. The line will pass
     * through both points
     * @param pointA First point laying on this line
     * @param pointB Second point laying on this line
     */    
    public final void setParametersFromPairOfPoints(Point2D pointA, 
            Point2D pointB){
        try{
            setParametersFromPairOfPoints(pointA, pointB, true);
        }catch(CoincidentPointsException ignore){} //exception never raised 
                                                //because of last true parameter
    }
    
    /**
     * Sets parameters of a 2D line from one point and its director vector
     * @param point point passing through the line
     * @param vector director vector
     * @throws IllegalArgumentException raised if vector length is not 2
     */
    public final void setParametersFromPointAndDirectorVector(Point2D point,
            double[] vector) throws IllegalArgumentException{
        
        if(vector.length != INHOM_VECTOR_SIZE)
            throw new IllegalArgumentException();
        
        //normalize point to increase accuracy
        point.normalize();
        
        mA = vector[0];
        mB = vector[1];
        
        mC = -(mA * point.getHomX() + mB * point.getHomY()) / point.getHomW();
        
        mNormalized = false;
    }
    
    /**
     * Returns boolean indicating whether provided point lies within this line
     * (at a maximum distance of provided threshold)
     * @param point Point to be checked
     * @param threshold Threshold to determine whether the point lies inside the
     * line
     * @return True if point lies inside the line (is locus), false otherwise
     * @throws IllegalArgumentException Raised if provided threshold is negative
     */
    public boolean isLocus(Point2D point, double threshold) 
            throws IllegalArgumentException{
        if(threshold < MIN_THRESHOLD) throw new IllegalArgumentException();
        
        //make dot product of homogeneous coordinates with line
        //m = [x, y, w], l = [a, b, c], then
        //x * a + y * b + w * c must be very small to be locus
        
        point.normalize();
        normalize();
        
        double dotProd = point.getHomX() * mA + point.getHomY() * mB +
                point.getHomW() * mC;
        
        return Math.abs(dotProd) < threshold;
    }
    
    /**
     * Returns boolean indicating whether provided point lies within this line
     * (at a maximum distance of DEFAULT_LOCUS_THRESHOLD)
     * @param point Point to be checked
     * @return True if point lies inside the line (is locus), false otherwise
     */    
    public boolean isLocus(Point2D point){
        return isLocus(point, DEFAULT_LOCUS_THRESHOLD);
    }
    
    /**
     * Distance between a line and a point. Returned distance equals to the 
     * euclidean distance between this line and provided point but having sign.
     * Sign indicates whether point is at one side or the other of the line.
     * @param point Point whose distance to this line will be computed
     * @return Distance between this line and provided point
     */
    public double signedDistance(Point2D point){
        point.normalize();
        normalize();
        
        //numerator is the dot product of point and line
        double num = point.getHomX() * mA + point.getHomY() * mB + 
                point.getHomW() * mC;
        
        double den = Math.sqrt(mA * mA + mB * mB) * point.getHomW();
        
        return num / den;
    }
    
    /**
     * Returns the point belonging to this line closest to provided point, which
     * will be located at signedDistance(Point2D) from this line.
     * If provided point belong to this line, then the same point will be 
     * returned as a result
     * @param point Point to be checked
     * @return Closest point
     */
    public Point2D getClosestPoint(Point2D point){
        return getClosestPoint(point, DEFAULT_LOCUS_THRESHOLD);
    }

    /**
     * Returns the point belonging to this line closest to provided point, which
     * will be located at signedDistance(Point2D) from this line.
     * If provided point belong to this line, then the same point will be 
     * returned as a result
     * @param point Point to be checked
     * @param threshold Threshold to determine whether point is locus of line or
     * not
     * @return Closest point
     * @throws IllegalArgumentException Raised if threshold is negative
     */    
    public Point2D getClosestPoint(Point2D point, double threshold)
            throws IllegalArgumentException{
        Point2D result = Point2D.create();
        closestPoint(point, result, threshold);
        return result;
    }
    
    /**
     * Computes the point belonging to this line closest to provided point, 
     * which will be located at signedDistance(Point2D) from this line.
     * If provided point belongs to this line, then the same point will be
     * returned as a result
     * @param point Point to be checked
     * @param result Instance where the closest point will be stored
     */
    public void closestPoint(Point2D point, Point2D result){
        closestPoint(point, result, DEFAULT_LOCUS_THRESHOLD);
    }

    /**
     * Computes the point belonging to this line closest to provided point, 
     * which will be located at signedDistance(Point2D) from this line.
     * If provided point belongs to this line, then the same point will be
     * returned as a result
     * @param point Point to be checked
     * @param result Instance where the closest point will be stored
     * @param threshold threshold to determine whether a point belongs to the
     * line's locus
     * @throws IllegalArgumentException Raised if threshold is negative
     */    
    public void closestPoint(Point2D point, Point2D result, double threshold)
            throws IllegalArgumentException{
        if(threshold < MIN_THRESHOLD) throw new IllegalArgumentException();
        
        //normalize point to increase accuracy
        point.normalize();
        
        if(isLocus(point, threshold)){
            //if point belongs to line, then it is returned as result
            result.setCoordinates(point);
            return;
        }
        
        //move point in director vector direction until it belong to this line
        //(point.getInhomX() + mA * amount) * mA + (point.getInhomY() + 
        //mB * amount) * mB + mC = 0
        
        double amount = -(point.getHomX() * mA + point.getHomY() * mB + 
                point.getHomW() * mC) / 
                (point.getHomW() * (mA * mA + mB * mB));
        result.setHomogeneousCoordinates(point.getHomX() + 
                mA * amount * point.getHomW(), 
                point.getHomY() + mB * amount * point.getHomW(),
                point.getHomW());
        result.normalize();
    }
    
    /**
     * Returns parameters of this line as an array containing [a, b, c]
     * @return Array containing all the parameters that describe this line
     */
    public double[] asArray(){
        double[] array = new double[LINE_NUMBER_PARAMS];
        asArray(array);
        return array;
    }
    
    /**
     * Stores the parameters of this line in provided array as [a, b, c].
     * @param array Array where parameters of this line will be stored
     * @throws IllegalArgumentException Raised if provided array doesn't have
     * length 3
     */
    public void asArray(double[] array) throws IllegalArgumentException{
        if(array.length != LINE_NUMBER_PARAMS)
            throw new IllegalArgumentException();
        
        array[0] = mA;
        array[1] = mB;
        array[2] = mC;        
    }
    
    /**
     * Normalizes the parameters of this line to increase the accuracy of some
     * computations
     */
    public void normalize(){
        if(!mNormalized){
            double norm = Math.sqrt(mA * mA + mB * mB + mC * mC);
            
            if(norm > PRECISION){
                mA /= norm;
                mB /= norm;
                mC /= norm;
            
                mNormalized = true;
            }
        }
    }
    
    /**
     * Returns boolean indicating whether this line has already been normalized.
     * @return True if this line is normalized, false otherwise
     */
    public boolean isNormalized(){
        return mNormalized;
    }
    
    /**
     * Returns director vector of this line
     * @return Director vector of this line
     */
    public double[] getDirectorVector(){
        double[] out = new double[INHOM_VECTOR_SIZE];
        directorVector(out);
        return out;
    }
    
    /**
     * Computes director vector of this line and stores the result in provided 
     * array
     * @param directorVector Array containing director vector
     * @throws IllegalArgumentException Raised if provided array does not have
     * length 2
     */
    public void directorVector(double[] directorVector) 
            throws IllegalArgumentException{
        if(directorVector.length != INHOM_VECTOR_SIZE) 
            throw new IllegalArgumentException();
        
        directorVector[0] = mA;
        directorVector[1] = mB;
    }
    
    /**
     * Computes the intersection of this line with provided line.
     * Notice that parallel lines intersect at infinity.
     * @param otherLine other line to be intersected with
     * @return A 2D point indicating containing the intersection
     * @throws NoIntersectionException if for numerical instabilities the 
     * intersection cannot be computed.
     */
    public Point2D getIntersection(Line2D otherLine) 
            throws NoIntersectionException{
        Point2D result = Point2D.create();
        intersection(otherLine, result);
        return result;
    }
    
    /**
     * Computes the intersection of this line with provided line.
     * Notice that parallel lines intersect at infinity.
     * @param otherLine other line to be intersected with
     * @param result 2D point where intersection will be stored. For greater
     * accuracy it is recommended to use an HomogeneousPoint2D instance.
     * @throws NoIntersectionException if for numerical instabilities the 
     * intersection cannot be computed.
     */
    public void intersection(Line2D otherLine, Point2D result) 
            throws NoIntersectionException{
        
        //normalize lines to increase accuracy
        normalize();
        otherLine.normalize();
        
        //set matrix where each row contains the parameters of the line
        try{
            Matrix m = new Matrix(2, 3);
            m.setElementAt(0, 0, mA);
            m.setElementAt(0, 1, mB);
            m.setElementAt(0, 2, mC);
        
            m.setElementAt(1, 0, otherLine.getA());
            m.setElementAt(1, 1, otherLine.getB());
            m.setElementAt(1, 2, otherLine.getC());
        
            //If lines are not parallel, then matrix has rank 2, and its right 
            //null-space is equal to their intersection.
            SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();
        
            //lines are parallel
            if(decomposer.getRank() < 2) throw new NoIntersectionException();
        
            Matrix V = decomposer.getV();
        
            //last column of V contains the right nullspace of m, which is the
            //intersection of lines expressed in homogeneous coordinates.
            //because column is already normalized by SVD decomposition, point
            //will also be normalized
            result.setHomogeneousCoordinates(V.getElementAt(0, 2), 
                    V.getElementAt(1, 2), V.getElementAt(2, 2));
        }catch(AlgebraException e){
            //lines are numerically unstable
            throw new NoIntersectionException(e);
        }
    }
    
    /**
     * Computes the dot product between the parameters A, B,C of this line and
     * the ones of provided line.
     * This method normalizes both lines to compute dot product
     * @param line line to compute dot product with
     * @return dot product value
     */
    public double dotProduct(Line2D line){
        normalize();
        line.normalize();
        return mA * line.mA + mB * line.mB + mC * line.mC;
    }
    
    /**
     * Checks if the line described by this instance equals provided line up to
     * provided threshold
     * @param line line to be compared to.
     * @param threshold threshold grade of tolerance to determine whether the
     * lines are equal or not. It is used because due to machine precision,
     * the values might not be exactly equal (if not provided
     * DEFAULT_COMPARISON_THRESHOLD is used)
     * @return true if current line and provided one are the same, false
     * otherwise
     * @throws IllegalArgumentException if threshold is negative
     */
    public boolean equals(Line2D line, double threshold)
            throws IllegalArgumentException{
        
        if(threshold < MIN_THRESHOLD) throw new IllegalArgumentException();
        
        normalize();
        line.normalize();
        
        return (1.0 - Math.abs(dotProduct(line))) <= threshold;
    }
    
    /**
     * Checks if the line described by this instance equals provided line up to
     * default comparison threshold
     * @param line line to be compared to.
     * @return true if current line and provided one are the same, false 
     * otherwise
     */
    public boolean equals(Line2D line){
        return equals(line, DEFAULT_COMPARISON_THRESHOLD);
    }
    
    /**
     * Checks if provided object equals current line
     * @param obj object to compare
     * @return true if both objects are considered to be equal, false otherwise
     */
    @Override
    public boolean equals(Object obj){
        if(!(obj instanceof Line2D)) return false;
        if(obj == this) return true;
        
        Line2D line = (Line2D)obj;
        return equals(line);
    }    

    /**
     * Returns hash code value. This is only defined to keep the compiler happy.
     * This method must be overridden in subclasses of this class.
     * @return Hash code
     */        
    @Override
    public int hashCode() {
        int hash = 7;
        hash = 73 * hash + (int) (Double.doubleToLongBits(this.mA) ^ 
                (Double.doubleToLongBits(this.mA) >>> 32));
        hash = 73 * hash + (int) (Double.doubleToLongBits(this.mB) ^ 
                (Double.doubleToLongBits(this.mB) >>> 32));
        hash = 73 * hash + (int) (Double.doubleToLongBits(this.mC) ^ 
                (Double.doubleToLongBits(this.mC) >>> 32));
        hash = 73 * hash + (this.mNormalized ? 1 : 0);
        return hash;
    }
    
    /**
     * Creates a new instance of a 2D line located at the canonical infinity.
     * The canonical infinity corresponds to all 2D points located at infinity
     * (i.e. m = (x,y,w = 0), hence l = (A = 0, B = 0, C = 1))
     * @return a new instance of a 2D line located at the canonical infinity
     */
    public static Line2D createCanonicalLineAtInfinity(){
        Line2D l = new Line2D();
        setAsCanonicalLineAtInfinity(l);
        return l;
    }
    
    /**
     * Sets provided 2D line into the canonical infinity.
     * The canonical infinity corresponds to all 2D points located at infinity
     * (i.e. m = (x,y,w = 0), hence l = (A = 0, B = 0, C = 1))
     * @param line 2D line to be set at infinity
     */
    public static void setAsCanonicalLineAtInfinity(Line2D line){
        line.mA = line.mB = 0.0;
        line.mC = 1.0;
        line.mNormalized = true;
    }    
}
