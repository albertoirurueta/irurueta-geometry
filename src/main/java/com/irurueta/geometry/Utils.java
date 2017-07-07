/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.Utils
 * 
 * @author Alberto Irurueta (airurueta@visual-engin.com)
 * @date November 7, 2012
 */
package com.irurueta.geometry;

/**
 * Helper class containing commonly used methods related to geometry
 */
public class Utils {
    /**
     * Amount to add/subtract to an angle in degrees to make a half turn.
     */
    public static final double HALF_TURN_DEGREES = 180.0;
    
    /**
     * Amount to add/subtract to an angle in radians to make a half turn.
     */
    public static final double HALF_TURN_RADIANS = Math.PI;
    
    /**
     * Converts provided value from radians to degrees
     * @param radians Angle in radians
     * @return Converted angle in degrees
     */
    public static double convertToDegrees(double radians){
        return radians / Math.PI * 180.0;
    }
    
    /**
     * Converts provided value from degrees to radians
     * @param degrees Angle in degrees
     * @return Converted angle in radians
     */
    public static double convertToRadians(double degrees){
        return degrees / 180.0 * Math.PI;
    }   
        
    /**
     * Normalizes an angle between the range -180...180 by adding or subtracting
     * the required amount of turns.
     * @param angle angle to be normalized expressed in degrees.
     * @return the same angle but normalized.
     */
    public static double normalizeAngleDegrees(double angle){
        return normalizeAngle(angle, HALF_TURN_DEGREES);
    }
    
    /**
     * Normalizes an angle between the range -pi...pi by adding or subtracting 
     * the required amount of turns.
     * @param angle angle to be normalized expressed in degrees.
     * @return the same angle but normalized.
     */
    public static double normalizeAngleRadians(double angle){
        return normalizeAngle(angle, HALF_TURN_RADIANS);
    }
    
    /**
     * Normalizes an angle between the range -pi...pi (or -180...180) by adding
     * or subtracting the required amount of turns.
     * This method can work with angles expressed either on degrees or radians
     * depending on the units used for the definition of a half turn.
     * @param angle angle to be normalized.
     * @param halfTurn the definition of a half turn (either in degrees or 
     * radians).
     * @return the same angle but normalized.
     */
    private static double normalizeAngle(double angle, double halfTurn){
        double result = angle;
        double fullTurn = 2.0 * halfTurn;
        
        if(result <= -halfTurn){
            int nTurns = (int)Math.ceil(-result / fullTurn);
            result += nTurns * fullTurn;
            if(result > halfTurn){
                result -= fullTurn;
            }
        }
        if(result > halfTurn){
            int nTurns = (int)Math.ceil(result / fullTurn);
            result -= nTurns * fullTurn;
            if(result <= -halfTurn){
                result += fullTurn;
            }
        }
        return result;
    }
}
