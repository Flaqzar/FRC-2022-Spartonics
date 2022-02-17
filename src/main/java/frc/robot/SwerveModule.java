package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

/**
 * represents a swerve module. Wraps two falcon500s, one for driving and one for steering
 */
public class SwerveModule
{
	// Motor objects used in each swerve drive module
	WPI_TalonFX driveFalcon;
	WPI_TalonFX steeringFalcon;
	double lastAngle;
	WPI_CANCoder canCoder;
	double offset;

	/** The PID id used to determine what PID settings to use */
	private static final int PID_ID = 0;
	/** Rotation measured in radians. */
	private double currentRotation;

	/**
	 * Initialize new swerve module.
	 * 
	 * @param driveMotor drive motor id
	 * @param steeringMotor steering motor id
	 * @param canCoder canCoder id
	 */
	public SwerveModule(int driveMotor, int steeringMotor, int canCoder, double offsetIn)
	{
		// get the motor objects from the CAN bus
		this.driveFalcon = new WPI_TalonFX(driveMotor);
		this.steeringFalcon = new WPI_TalonFX(steeringMotor);
		this.canCoder = new WPI_CANCoder(canCoder);
		this.currentRotation = 0;
		//this.offset = offsetIn * 26214.4d / 360d;
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
		
		this.canCoder.configFactoryDefault();
		
		this.canCoder.setPositionToAbsolute();
		
		// reset angle
		System.out.println("Before: ");
		System.out.println("  Can Coder: " + this.canCoder.getDeviceID());
		System.out.println("  Can Rotation: " + this.canCoder.getAbsolutePosition());
		System.out.println("  Motor Rotation: " + this.steeringFalcon.getActiveTrajectoryPosition());

		this.steeringFalcon.set(ControlMode.Position, this.canCoder.getAbsolutePosition() * 26214.4d / 360d);
		this.currentRotation = this.steeringFalcon.getActiveTrajectoryPosition() * Constants.TWO_PI / 26214.4d;
		
		System.out.println("After: ");
		System.out.println("  Can Coder: " + this.canCoder.getDeviceID());
		System.out.println("  Can Rotation: " + this.canCoder.getAbsolutePosition());
		System.out.println("  Motor Rotation: " + this.steeringFalcon.getActiveTrajectoryPosition());
		System.out.println();
	}

	/**
	 * Sets the angle of the motor.
	 * Calculations by Alex: https://www.desmos.com/calculator/t9mc7gj1bf
	 * @param angle the angle in radians
	 */
	public void setAngle(double angle)
	{
		// Make sure that the input angle is a real number.
		if(!Double.isNaN(angle))
		{
			// Clamps the motor's rotation from 0 - 2π
			double motorAngle = this.currentRotation % Constants.TWO_PI;
			// Adds the two angles' difference to the motor's current rotation
			this.currentRotation -= motorAngle - angle + (Math.abs(motorAngle - angle) > Math.PI ? (motorAngle > angle ? -Constants.TWO_PI : Constants.TWO_PI) : 0d) + Constants.PI_OVER_TWO;
		}
		
		// Sets the new rotation
 		this.steeringFalcon.set(ControlMode.Position, this.currentRotation / Constants.TWO_PI * 26214.4d);
	}

	public void setSpeed(double speed)
	{
		if (speed < 0 || speed > 1) {
			System.out.println("Warning - SwerveModule.setSpeed: speed outside acceptable range.");
			return;
		}

		this.driveFalcon.set(speed);
	}

	public void stop()
	{
		this.driveFalcon.stopMotor();
	}

	public double getCanRotation()
	{
		return this.canCoder.getAbsolutePosition();
	}

	/**
	 * Converts a joystick's x and y coordinates into a radian angle from 0 - 2π.
	 * @param x the joystick's x position
	 * @param y the joystick's y position
	 * @return The joystick's rotation in radians.
	 */
	public static double convertJoystickToAngle(double x, double y)
	{
		return Math.atan(y / x) + (x < 0d ? Math.PI : 0d) + Constants.PI_OVER_TWO;
	}
}
