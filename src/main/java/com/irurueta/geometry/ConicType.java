/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.ConicType
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 2, 2012
 */
package com.irurueta.geometry;

/**
 * Enumerator that indicates the type of conic depending on the values of its
 * inner parameters
 */
public enum ConicType {
    /**
     * Conic parameters satisfying b^2 - ac &lt; 0.
     */
   ELLIPSE_CONIC_TYPE,
   
   /**
    * Conic parameters satisfying b^2 - ac &lt; 0 and a = c, b = 0
    */
   CIRCLE_CONIC_TYPE,
   
   /**
    * Conic parameters satisfying b^2 - ac = 0.
    */
   PARABOLA_CONIC_TYPE,
   
   /**
    * Conic parameters satisfying b^2 - ac &lt; 0.
    */
   HYPERBOLA_CONIC_TYPE,
   
   /**
    * Conic parameters satisfying b^2 - ac &lt; 0 and a + b = 0.
    */
   RECTANGULAR_HYPERBOLA_CONIC_TYPE
}
