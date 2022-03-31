package frc.robot;

public class Constants
{
	/**	Prevents accidental instantiation of Constants class. (sonarlint java:S1118)*/
	private Constants() {
		throw new IllegalStateException("Utility class");
	  }
	/**
	 * The delay in milliseconds before a report gets sent to DriverStation if an
	 * action fails
	 */
	public static final int MS_DELAY = 30;

	/** Settings are {kF, kP, kI, kD} */ // F P I D
	protected static final double[] PID_SETTINGS = { 0d, 0.15d, 0d, 1.50d};

	/**2π*/
	public static final double TWO_PI = 2d * Math.PI;

	/**π/2*/
	public static final double PI_OVER_TWO = Math.PI / 2d;

	/** number of inches each drive encoder tick is */
	public static final double INCHES_PER_TICK = Math.PI/3456d;
	/** number of ticks in one inch */
	public static final double TICKS_PER_INCH = 3456d/Math.PI;; 
}
