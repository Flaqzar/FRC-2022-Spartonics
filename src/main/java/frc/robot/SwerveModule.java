package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * represents a swerve module. Wraps two falcon500s, one for driving and one for
 * steering
 */
public class SwerveModule
{
	// Motor objects used in each swerve drive module
	WPI_TalonFX driveFalcon;
	WPI_TalonFX steeringFalcon;
	double lastAngle;
	// TODO: add the CANcoder

	/** The PID id used to determine what PID settings to use */
	private static final int PID_ID = 0;


	public SwerveModule(int driveMotor, int steeringMotor)
	{
		// get the motor objects from the CAN bus
		this.driveFalcon = new WPI_TalonFX(driveMotor);
		this.steeringFalcon = new WPI_TalonFX(steeringMotor);
	}

	/**
	 * initialize the motors. Call this before doing stuff with the motor.
	 */
	public void init()
	{
		// Motor settings stuff
		this.steeringFalcon.configFactoryDefault();
		this.steeringFalcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, PID_ID, Constants.MS_DELAY);
		this.steeringFalcon.setSensorPhase(true);
		this.steeringFalcon.setInverted(false);

		// no idea what this does.
		this.steeringFalcon.configNominalOutputForward(0d, Constants.MS_DELAY);
		this.steeringFalcon.configNominalOutputReverse(0d, Constants.MS_DELAY);
		this.steeringFalcon.configPeakOutputForward(1d, Constants.MS_DELAY);
		this.steeringFalcon.configPeakOutputReverse(-1d, Constants.MS_DELAY);

		this.steeringFalcon.configAllowableClosedloopError(0, PID_ID, Constants.MS_DELAY);

		// configure PID tuning
		this.steeringFalcon.config_kF(PID_ID, Constants.PID_SETTINGS[0], Constants.MS_DELAY);
		this.steeringFalcon.config_kP(PID_ID, Constants.PID_SETTINGS[1], Constants.MS_DELAY);
		this.steeringFalcon.config_kI(PID_ID, Constants.PID_SETTINGS[2], Constants.MS_DELAY);
		this.steeringFalcon.config_kD(PID_ID, Constants.PID_SETTINGS[3], Constants.MS_DELAY);

		// reset angle
		this.steeringFalcon.set(ControlMode.Position, 0);
	}

	/**
	 * set the angle of the motor
	 * 
	 * @param angle desired angle of the motor
	 */
	public void setAngle(double angle)
	{
		/*
		 * equations here are taken from:
		   https://gamedev.stackexchange.com/questions/14900/turning-a-sprite-such-that-it-rotates-in-the-direction-thats-most-efficient
		   https://gamedev.stackexchange.com/questions/46552/360-degree-rotation-skips-back-to-0-degrees-when-using-math-atan2y-x
		   https://github.com/Flaqzar/FRC-2022-Spartonics/pull/5/files/19c5107927d95797d443cb8c72289a7723397561#r796195340
		 */
		if (angle > this.lastAngle + 180)
		{ // if the angle is more than 180 degrees from the last angle, subtract 360 degrees to bring the target angle closer
			angle -= 360;
		}
		else if (angle < this.lastAngle - 180)
		{ // add 360 degrees if the angle is less than a -180 degree turn (absolute
												// value)
			angle += 360;
		} // if none of the above are covered, that means that the target angle is within
			// 180 degrees of the old angle, and we do not need to modify it.
			// if the difference between the current angle of the motor and the desired
			// angle of the motor, while going clockwise, is less than 180, rotate clockwise
		if (Math.abs(this.lastAngle - angle) < 180)
		{
			this.steeringFalcon.set(ControlMode.Position, (360 - angle) / 360 * 2048);
		}

		this.lastAngle = angle;
	}
}
