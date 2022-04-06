package frc.robot.math;

/**
 * Container for various static variables.
 * 
 * @author 2141 Spartonics
 */
public class Constants
{
	/**
	 * The delay in milliseconds before a report gets sent to DriverStation if an
	 * action fails.
	 */
	public static final int MS_DELAY = 30;

	/** Settings are {kF, kP, kI, kD} */
	public static final double[] PID_SETTINGS = { 0d, 0.15d, 0d, 1.50d};

	/** 2π */
	public static final double TWO_PI = 2d * Math.PI;

	/** π/2 */
	public static final double PI_OVER_TWO = Math.PI / 2d;

    /** Number of inches each drive encoder tick is. */
    public static final double INCHES_PER_TICK = Math.PI / 3456d;

    /** Number of ticks in one inch. */
    public static final double TICKS_PER_INCH = 3456d / Math.PI;
}
