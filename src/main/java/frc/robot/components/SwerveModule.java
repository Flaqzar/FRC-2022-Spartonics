package frc.robot.components;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import frc.robot.math.Constants;
import frc.robot.math.Vec2d;

/**
 * Container for one swerve module. Wraps two falcon500s: one for driving and one for steering.
 * 
 * @author 2141 Spartonics
 */
public class SwerveModule
{
	/** The PID id used to determine what PID settings to use. */
	private static final int PID_ID = 0;
	/** The motor controlling the module's movement. */
	private final WPI_TalonFX drivingMotor;
	/** The motor controlling the module's rotation. */
	private final WPI_TalonFX steeringMotor;
	/** The can coder measuring the module's absolute rotaiton. */
	private final WPI_CANCoder canCoder;
	/** The can coder's rotational offset. This value must be manually set through phoenix tuner. */
	private final double canOffset;
	/** The direction of rotation relative to the center of the drivetrain. */
	private final double rotDireciton;
	/** The steering motor rotation measured in radians. */
	private double motorRotation;
	/** The speed multiplier used to invert the speed of the drive motor in {@link #setMotion(Vec2d) setMotion}. */
	private int speedMultiplier;

	/**
	 * @param driveMotor driving motor ID
	 * @param steeringMotor steering motor ID
	 * @param canCoder can coder ID
	 * @param rotationDirection the steering motor's rotational direction, usually perpendicular to the center of the robot
	 * @param canCoderOffset the can coder's rotational offset
	 */
	public SwerveModule(int driveMotorID, int steeringMotorID, int canCoderID, double rotationDirection, double canCoderOffset)
	{
		this.drivingMotor = new WPI_TalonFX(driveMotorID);
		this.steeringMotor = new WPI_TalonFX(steeringMotorID);
		this.canCoder = new WPI_CANCoder(canCoderID);
		this.canOffset = canCoderOffset;
		this.rotDireciton = rotationDirection;
		this.motorRotation = 0;

		this.speedMultiplier = 1;
	}

	/**
	 * Configures the motors and sets the steering motor's rotation to zero.
	 */
	public void init()
	{
		// Reset the steering motor.
		this.steeringMotor.configFactoryDefault(Constants.MS_DELAY);

		// Miscellaneous settings.
		this.steeringMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, PID_ID, Constants.MS_DELAY);
		this.steeringMotor.setSensorPhase(true);
		this.steeringMotor.setInverted(false);
		this.steeringMotor.configNominalOutputForward(0d, Constants.MS_DELAY);
		this.steeringMotor.configNominalOutputReverse(0d, Constants.MS_DELAY);
		this.steeringMotor.configPeakOutputForward(1d, Constants.MS_DELAY);
		this.steeringMotor.configPeakOutputReverse(-1d, Constants.MS_DELAY);
		this.steeringMotor.configAllowableClosedloopError(0, PID_ID, Constants.MS_DELAY);

		// PID tune the steering motor.
		this.steeringMotor.config_kF(PID_ID, Constants.PID_SETTINGS[0], Constants.MS_DELAY);
		this.steeringMotor.config_kP(PID_ID, Constants.PID_SETTINGS[1], Constants.MS_DELAY);
		this.steeringMotor.config_kI(PID_ID, Constants.PID_SETTINGS[2], Constants.MS_DELAY);
		this.steeringMotor.config_kD(PID_ID, Constants.PID_SETTINGS[3], Constants.MS_DELAY);

		// Configure the can coder.
		this.canCoder.configFactoryDefault(Constants.MS_DELAY);
		this.canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, Constants.MS_DELAY);
		this.canCoder.configMagnetOffset(this.canOffset, Constants.MS_DELAY);
		this.canCoder.setPositionToAbsolute(Constants.MS_DELAY);

		// Reset the motor rotations.
		this.reset();
	}

	/**
	 * Sets steering motor's rotation to zero.
	 */
	public void reset()
	{
		// Set the steering motor's internal rotation to 0.
		this.steeringMotor.setSelectedSensorPosition(0d);
		// The angle to rotate to face forward.
		double angleToRotate = this.canCoder.getAbsolutePosition() > 180d ? this.canCoder.getAbsolutePosition() - 360d : this.canCoder.getAbsolutePosition();
		// Set the steering motor's rotation.
		this.steeringMotor.set(ControlMode.Position, -angleToRotate * Constants.DEG_TO_TICK * 12.8d);
		// Reset the steering motor's internal rotation to 0.
		this.steeringMotor.setSelectedSensorPosition(0d);
		// Reset the stored motor rotation variable.
		this.motorRotation = 0d;
	}

	/**
	 * The old motor rotation function. Simply rotates the steering motor in the direction of 
	 * the passed vector, and sets the driving motor's speed to the length of the vector. 
	 * The steering motor will take the shortest path of rotation by rotating a maximum of 
	 * 180° in either direction. IMPORTANT: To use this function, add 90° to the gyro's rotation 
	 * and subtract 90° from the module rotations in the SwerveDrive class. Calculations by Alex Green.
	 * 
	 * @param vec the vector representing the module's movement
	 * @deprecated in favor of {@link #setMotion(Vec2d) setMotion}.
	 * @see <a href="https://www.desmos.com/calculator/dgkniftpn6">Alex's Calculations</a>
	 */
	@Deprecated
	public void setAngle(Vec2d vec)
	{
		// Get the angle of the passed vector.
		double angle = vec.getAngle();

		// Only rotate if the angle isn't NaN.
		if(!Double.isNaN(angle))
		{
			// The steering motor's rotation bounded between 0 and 2π.
			double motorAngle = this.motorRotation % Constants.TWO_PI;
			// The distance between the motor's current rotation and the passed rotation.
			double angleDif = motorAngle - angle;
			// An overcomplicated way of getting the shortest distance to to the passed vector.
			this.motorRotation -= angleDif + (Math.abs(angleDif) > Math.PI ? (motorAngle > angle ? -Constants.TWO_PI : Constants.TWO_PI) : 0d) + Constants.PI_OVER_TWO;
		}

		// Update the two motor's with the new values.
 		this.steeringMotor.set(ControlMode.Position, this.motorRotation / Constants.TWO_PI * 26214.4d);
		this.drivingMotor.set(ControlMode.PercentOutput, vec.getLength());
	}

	/**
	 * Rotates the steering motor in the direction of the passed vector, and sets the driving 
	 * motor's speed to the length of the vector. The steering motor will take the shortest 
	 * path of rotation by rotating a maximum of 90° in either direction. If the distance between 
	 * the current motor rotation and the passed vector's rotation is greater than 90°, it will 
	 * rotate away from the vector and invert the speed, effectively rotating the motor in the 
	 * direction of the vector. Calculations by Alex Green.
	 * 
	 * @param vec the vector representing the serve module's movement
	 * @see <a href="https://www.desmos.com/calculator/ip3i9zr1qg">Alex's Calculations</a>
	 */
	public void setMotion(Vec2d vec)
	{
		// Only rotate if the vector's length is greater than 0 to prevent NaN values.
		if(vec.getLengthSquared() != 0d)
		{
			// Turn the steering motor's rotation into a vector.
			Vec2d motorVec = new Vec2d(this.motorRotation, false);

			// Invert the driving motor if the wheel is facing more than 90° away from the target angle.
			this.speedMultiplier = vec.distanceTo(motorVec) > Constants.PI_OVER_TWO ? -1 : 1;

			// Get the angles from the two vectors.
			double motorAngle = motorVec.getAngle();
			double vecAngle = vec.getAngle();
			
			// Calculate all the possible rotations the steering motor could take.
			Double[] angles = new Double[5];
			angles[0] = vecAngle - motorAngle;
			angles[1] = angles[0] + Constants.TWO_PI;
			angles[2] = angles[0] - Constants.TWO_PI;
			angles[3] = angles[0] + Math.PI;
			angles[4] = angles[0] - Math.PI;

			// Sort the array to get the shortest angle at the front.
			Arrays.sort(angles, (d1, d2) -> Double.compare(Math.abs(d1), Math.abs(d2)));
			
			// Set the steering motor's rotation.
			this.motorRotation += angles[0];
			this.steeringMotor.set(ControlMode.Position, this.motorRotation * Constants.RAD_TO_TICK * 12.8d);
		}

		// Set the driving motor's speed.
		this.drivingMotor.set(ControlMode.PercentOutput, vec.getLength() * this.speedMultiplier);
	}

	/**
	 * Getter for the drive motor's rotation direction.
	 * 
	 * @return The drive motor
	 */
	public double getRotationDirection()
	{
		return this.rotDireciton;
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

	@Override
	public String toString()
	{
		// The class will be represented as "SwerveModule[Steering Motor ID = ?, Driving Motor ID = ?, Cancoder ID = ?]"
		return "SwerveModule[Steering Motor ID = " + this.steeringMotor.getDeviceID() + ", Driving Motor ID = " + this.drivingMotor.getDeviceID() + ", Cancoder ID = " + this.canCoder.getDeviceID() + "]";
	}
}
