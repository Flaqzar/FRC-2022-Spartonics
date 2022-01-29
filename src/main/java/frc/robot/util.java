package frc.robot;

public class util
{
  private util()
  { // prevents accidental instantiation of util
    throw new IllegalStateException("Utility Class");
  }

  /**
   * Takes two numbers (x and y positions of the stick) and returns the angle that the stick is facing, in degrees in range (0, 360].
   * <p>Expects both 'x' and 'y' to be floats from -1 to 1, with (0,0) when the stick is centered
   * @param x x value of stick
   * @param y y value of stick
   * @return angle of stick in degrees
   */
  
  public static double XYposToDeg(double x, double y)
  {
    double a_tan_val = Math.toDegrees(Math.atan(y/x));

    if (x < 0 && y < 0) { // third quadrant, but calculated as first quadrant because negative/negative=positive.
      a_tan_val += 180.0;
    }
    
    // in theory, a_tan_val should be in range [-180, 180], the while loops are a failsafe to ensure angle is in range
    while (a_tan_val <= 0) {
      a_tan_val += 360.0;
    }
    while (a_tan_val > 360) {
      a_tan_val -= 360.0;
    }

    return a_tan_val;
  }
}
