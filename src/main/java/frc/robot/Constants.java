package frc.robot;

public class Constants
{
	/**
	 * The delay in milliseconds before a report gets sent to DriverStation if an
	 * action fails
	 */
	public static final int MS_DELAY = 30;

	/** Settings are {kF, kP, kI, kD} */ // F P I D
	public static final double[] PID_SETTINGS = { 0d, 0.25d, 0d, 1d };
}
