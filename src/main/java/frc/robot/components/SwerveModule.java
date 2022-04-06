package frc.robot.components;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import frc.robot.math.Constants;
import frc.robot.math.Vec2d;

/**
 * Represents a swerve module. Wraps two falcon500s, one for driving and one for steering
 */
public class SwerveModule
{
	private final WPI_TalonFX drivingMotor;
	private final WPI_TalonFX steeringMotor;
	private final WPI_CANCoder canCoder;
	private final double canOffset;
	private final double rotDir;

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
	 * @param rotationDirection the motor's rotation direction, usually perpendicular to the center
	 * @param canCoderOffset the canCoder's rotation offset
	 */
	public SwerveModule(int driveMotorID, int steeringMotorID, int canCoderID, double rotationDirection, double canCoderOffset)
	{
		this.drivingMotor = new WPI_TalonFX(driveMotorID);
		this.steeringMotor = new WPI_TalonFX(steeringMotorID);
		this.canCoder = new WPI_CANCoder(canCoderID);
		this.canOffset = canCoderOffset;
		this.motorRotation = 0;
		this.rotDir = rotationDirection;
	}

	/**
	 * Configures the motors and sets the steering motor's rotation to zero.
	 */
	public void init()
	{
		// Motor settings stuff.
		this.steeringMotor.configFactoryDefault(Constants.MS_DELAY);
		this.steeringMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, PID_ID, Constants.MS_DELAY);
		this.steeringMotor.setSensorPhase(true);
		this.steeringMotor.setInverted(false);
		this.steeringMotor.setNeutralMode(NeutralMode.Brake);
		this.drivingMotor.setNeutralMode(NeutralMode.Brake);

		// Idk.
		this.steeringMotor.configNominalOutputForward(0d, Constants.MS_DELAY);
		this.steeringMotor.configNominalOutputReverse(0d, Constants.MS_DELAY);
		this.steeringMotor.configPeakOutputForward(1d, Constants.MS_DELAY);
		this.steeringMotor.configPeakOutputReverse(-1d, Constants.MS_DELAY);

		this.steeringMotor.configAllowableClosedloopError(0, PID_ID, Constants.MS_DELAY);

		// Steering falcon PID tuning.
		this.steeringMotor.config_kF(PID_ID, Constants.PID_SETTINGS[0], Constants.MS_DELAY);
		this.steeringMotor.config_kP(PID_ID, Constants.PID_SETTINGS[1], Constants.MS_DELAY);
		this.steeringMotor.config_kI(PID_ID, Constants.PID_SETTINGS[2], Constants.MS_DELAY);
		this.steeringMotor.config_kD(PID_ID, Constants.PID_SETTINGS[3], Constants.MS_DELAY);

		// Configure the can coder.
		this.canCoder.configFactoryDefault(Constants.MS_DELAY);
		this.canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, Constants.MS_DELAY);
		this.canCoder.configMagnetOffset(this.canOffset, Constants.MS_DELAY);
		this.canCoder.setPositionToAbsolute(Constants.MS_DELAY);

		// Reset the motor rotation
		this.reset();
	}

	/**
	 * Sets steering motor's rotation to zero.
	 */
	public void reset()
	{
		// Re-align the steering motor
		this.steeringMotor.setSelectedSensorPosition(0d);
		double angleToRotate = this.canCoder.getAbsolutePosition() > 180d ? this.canCoder.getAbsolutePosition() - 360d : this.canCoder.getAbsolutePosition();
		this.steeringMotor.set(ControlMode.Position, -angleToRotate * 26214.4d / 360d);
		this.steeringMotor.setSelectedSensorPosition(0d);
		this.motorRotation = 0d;
	}

	/**
	 * Sets the angle of the steering motor.
	 * Calculations by Alex: https://www.desmos.com/calculator/t9mc7gj1bf
	 * 
	 * @param angle the angle in radians.
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

 		this.steeringMotor.set(ControlMode.Position, this.motorRotation / Constants.TWO_PI * 26214.4d);
	}

	/**
	 * Sets the speed of the drive motor.
	 * 
	 * @param speed the percent speed from -1 to 1.
	 */
	public void setSpeed(double speed)
	{
		// Clamp the speed between -1 and 1.
		this.drivingMotor.set(Math.max(-1d, Math.min(1d, speed)));
	}

    /**
     * Resets the driving motor's encoder rotation to 0.
     */
    public void resetDriveEncoder()
    {
        this.drivingMotor.setSelectedSensorPosition(0);
    }

	/**
	 * Getter for the drive motor.
	 * 
	 * @return The drive motor
	 */
	public WPI_TalonFX getDriveMotor()
	{
		return this.drivingMotor;
	}

	/**
	 * Getter for the steering motor.
	 * 
	 * @return The steering motor
	 */
	public WPI_TalonFX getSteeringMotor()
	{
		return this.steeringMotor;
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

	/**
	 * Getter for the module's rotation direction.
	 * 
	 * @return The module's rotation direction.
	 */
	public double getRotationDirection()
	{
		return this.rotDir;
	}
}
