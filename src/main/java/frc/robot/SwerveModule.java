package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

/**
 * represents a swerve module. Wraps two falcon500s, one for driving and one for steering
 */
public class SwerveModule
{
	public final WPI_TalonFX drivingFalcon;
	private final WPI_TalonFX steeringFalcon;
	private final WPI_CANCoder canCoder;
	private final double canOffset;

	/** The steering motor rotation measured in radians. */
	public double motorRotation;

	/** The PID id used to determine what PID settings to use. */
	private static final int PID_ID = 0;

	/**
	 * Initialize new swerve module.
	 * 
	 * @param driveMotor drive motor id
	 * @param steeringMotor steering motor id
	 * @param canCoder canCoder id
	 */
	public SwerveModule(int driveMotorID, int steeringMotorID, int canCoderID, double canCoderOffset)
	{
		this.drivingFalcon = new WPI_TalonFX(driveMotorID);
		this.steeringFalcon = new WPI_TalonFX(steeringMotorID);
		this.canCoder = new WPI_CANCoder(canCoderID);
		this.canOffset = canCoderOffset;
		this.motorRotation = 0;
	}
	public void resetDriveEncoder()
	{
		this.drivingFalcon.setSelectedSensorPosition(0, PID_ID, 0);
	}

	/// returns the distance this motor has driven in encoder ticks
	public double getDriveEncoderDistance()
	{
		return this.drivingFalcon.getSelectedSensorPosition(PID_ID);
	}

	/**
	 * Configures the motors and sets the steering motor's rotation to zero.
	 */
	public void init()
	{
		// Motor settings stuff
		this.steeringFalcon.configFactoryDefault(Constants.MS_DELAY);
		this.steeringFalcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, PID_ID, Constants.MS_DELAY);
		this.steeringFalcon.setSensorPhase(true);
		this.steeringFalcon.setInverted(false);


		// Make sure the motors brake
		this.steeringFalcon.setNeutralMode(NeutralMode.Brake);
		this.drivingFalcon.setNeutralMode(NeutralMode.Brake);

		// no idea what this does.
		this.steeringFalcon.configNominalOutputForward(0d, Constants.MS_DELAY);
		this.steeringFalcon.configNominalOutputReverse(0d, Constants.MS_DELAY);
		this.steeringFalcon.configPeakOutputForward(1d, Constants.MS_DELAY);
		this.steeringFalcon.configPeakOutputReverse(-1d, Constants.MS_DELAY);

		this.steeringFalcon.configAllowableClosedloopError(0, PID_ID, Constants.MS_DELAY);

		// Steering falcon PID tuning
		this.steeringFalcon.config_kF(PID_ID, Constants.PID_SETTINGS[0], Constants.MS_DELAY);
		this.steeringFalcon.config_kP(PID_ID, Constants.PID_SETTINGS[1], Constants.MS_DELAY);
		this.steeringFalcon.config_kI(PID_ID, Constants.PID_SETTINGS[2], Constants.MS_DELAY);
		this.steeringFalcon.config_kD(PID_ID, Constants.PID_SETTINGS[3], Constants.MS_DELAY);

		// Drive falcon PID tuning 
		this.steeringFalcon.config_kF(PID_ID, Constants.PID_SETTINGS[0], Constants.MS_DELAY);
		this.steeringFalcon.config_kP(PID_ID, Constants.PID_SETTINGS[1], Constants.MS_DELAY);
		this.steeringFalcon.config_kI(PID_ID, Constants.PID_SETTINGS[2], Constants.MS_DELAY);
		this.steeringFalcon.config_kD(PID_ID, Constants.PID_SETTINGS[3], Constants.MS_DELAY);



		//* set maximum velocity during autonomous: values are in encoder ticks per 100ms
		//* i.e. a setting of 1*Constants.TICKS_PER_INCH = 1 inch per 100ms or 10in per second.
		this.steeringFalcon.configMotionCruiseVelocity(1*Constants.TICKS_PER_INCH, Constants.MS_DELAY);

		// set maximum current
		



		// Configure the can coder
		this.canCoder.configFactoryDefault(Constants.MS_DELAY);
		this.canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, Constants.MS_DELAY);
		this.canCoder.configMagnetOffset(this.canOffset, Constants.MS_DELAY);
		this.canCoder.setPositionToAbsolute(Constants.MS_DELAY);

		

		//Reset the motor rotation
		this.reset();

		//set motor output to 10%
		this.steeringFalcon.set(ControlMode.PercentOutput, 0.1);
	}

	/**
	 * Sets steering motor's rotation to zero.
	 */
	public void reset()
	{
		// Re-align the steering motor
		/*this.steeringFalcon.setSelectedSensorPosition(0d);
		double angleToRotate = this.canCoder.getAbsolutePosition() > 180d ? this.canCoder.getAbsolutePosition() - 360d : this.canCoder.getAbsolutePosition();
		this.steeringFalcon.set(ControlMode.Position, -angleToRotate * 26214.4d / 360d);
		this.steeringFalcon.setSelectedSensorPosition(0d);
		this.motorRotation = 0d;*/

	}

	/**
	 * Sets the angle of the steering motor.
	 * Calculations by Alex: https://www.desmos.com/calculator/t9mc7gj1bf
	 * 
	 * @param angle the angle in radians
	 */
	public void setAngle(Vec2d vec)
	{
		double angle = vec.getAngle();

		if(!Double.isNaN(angle))
		{
			double motorAngle = this.motorRotation % Constants.TWO_PI;
			double angleDif = motorAngle - angle;
			this.motorRotation -= angleDif + (Math.abs(angleDif) > Math.PI ? (motorAngle > angle ? -Constants.TWO_PI : Constants.TWO_PI) : 0d) + Constants.PI_OVER_TWO;
		}

 		this.steeringFalcon.set(ControlMode.Position, this.motorRotation / Constants.TWO_PI * 26214.4d);
	}

	/**
	 * Sets the speed of the drive motor.
	 * 
	 * @param speed the percent speed from -1 to 1
	 */
	public void setSpeed(double speed)
	{
		//Sets the driving motor's speed to the passed in value clamped between -1 and 1.
		this.drivingFalcon.set(Math.max(-1d, Math.min(1d, speed)));
	}

	/**
	 * Shuts down the motor.
	 */
	public void stop()
	{
		//idk
		this.drivingFalcon.stopMotor();
	}

	/**
	 * Getter for the drive motor.
	 * 
	 * @return The drive motor
	 */
	public WPI_TalonFX getDriveMotor()
	{
		return this.drivingFalcon;
	}

	/**
	 * Getter for the steering motor.
	 * 
	 * @return The steering motor
	 */
	public WPI_TalonFX getSteeringMotor()
	{
		return this.steeringFalcon;
	}

	/**
	 * Getter for the can coder.
	 * 
	 * @return The can coder
	 */
	public WPI_CANCoder getCanCoder()
	{
		return this.canCoder;
	}
}
