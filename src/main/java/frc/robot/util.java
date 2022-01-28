package frc.robot;

public class util {
	private util() { // prevents accidental instantiation of util
		throw new IllegalStateException("Utility Class");
	}

    static double radian_conversion_multiplier = 180f/Math.PI;  /// number (180/pi) used to convert radians to integers
	
    /**
     * Takes two numbers (x and y positions of the stick) and returns the angle that the stick is facing, in degrees.
     * expects both ``x`` and ``y`` to be floats from -1 to 1, with (0,0) when the stick is centered
     * @param x x value of stick
     * @param y y value of stick
     * @return angle of stick in degrees
     */
    
    public static double XYposToRad(double x, double y){
        double a_tan_val = -(Math.atan(y/x)*radian_conversion_multiplier);
        if (x > 0){
            return a_tan_val+90.0;
            
        } else {
            return a_tan_val+270.0;
        }
    }
}
