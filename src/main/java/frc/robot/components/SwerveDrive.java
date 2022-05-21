package frc.robot.components;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.math.Constants;
import frc.robot.math.Vec2d;

/**
 * Container for the robot's swerve drivetrain. Wraps a gyroscope and swerve modules.
 * 
 * @author 2141 Spartonics
 */
public class SwerveDrive implements IControllerMovement
{
	/** A timer used to continually reset the motors. Prevents overshooting the rotation. */
	private static int resetTimer = 0;
	/** The gryoscope used for rotaiton measurements. */
	private final AHRS gyro;
	/** A list of all of the swerve modules on the drivetrain. */
	private final List<SwerveModule> modules;
	/** The minimum movement speed of the drivetrain. */
	private final double minMove;
	/** The maximum movement speed of the drivetrain. */
	private final double maxMove;
	/** The maximum rotational speed of the robot. */
	private final double maxRot;
	
	/**
	 * @param minSpeed minimum movement speed (0 to 1)
	 * @param maxSpeed maximum movement speed (0 to 1)
	 * @param maxRotation maximum rotational speed (0 to 1)
	 * @param gyroscope the swerve drive's gyroscope
	 * @param swerveModules the swerve drive's wheel modules
	 */
	public SwerveDrive(double minSpeed, double maxSpeed, double maxRotation, AHRS gyroscope, SwerveModule... swerveModules)
	{
		this.minMove = minSpeed;
		this.maxMove = maxSpeed;
		this.maxRot = maxRotation;
		this.gyro = gyroscope;
		this.modules = Collections.unmodifiableList(Arrays.asList(swerveModules));
	}

	/**
	 * Move's the swerve drive. The direction vector is automatically adjusted by the 
	 * gyroscope. Both the magnitude of the directional vector and the rotational speed 
	 * added together must be less than or equal to 1.
	 * 
	 * @param directionVec the direction and speed to travel (magnitude from -1 to 1)
	 * @param rotationalSpeed how fast the robot rotates (from -1 to 1)
	 */
	public void move(Vec2d directionVec, double rotationalSpeed)
	{
		// Check if the robot is able to move.
		if(this.canDrive())
		{
			// Vector representation of the speed and direction of the drivetrain.
			Vec2d driveVec = directionVec.rotate(this.gyro.getYaw() + 90d, true);
			
			this.modules.forEach(m ->
			{
				// Vector representation of the modules rotation.
				Vec2d rotVec = new Vec2d(m.getRotationDirection() + Constants.PI_OVER_TWO, rotationalSpeed, false);
				// Add the movement and rotation vectors together.
				Vec2d finalVec = driveVec.add(rotVec);
				// Set the modules movement to the combined vector.
				m.setMotion(finalVec);
			});
		}
		// Reset the motors if the timer isn't 0.
		else if(resetTimer > 0)
		{
			this.modules.forEach(m -> m.reset());
			resetTimer--;
		}
	}

	@Override
	public void move(XboxController... controllers)
	{
		// Joystick values.
		double leftXAxis = 0d;
		double leftYAxis = 0d;
		double rightXAxis = 0d;

		// Button values.
		boolean minusButton = false;
		boolean plusButton = false;
		double rTrigger = 0d;

		// Iterate through the controllers to get their combined inputs.
		for(int i = 0; i < controllers.length; i++)
		{
			leftXAxis = Math.abs(controllers[i].getRawAxis(0)) > Math.abs(leftXAxis) ? controllers[i].getRawAxis(0) : leftXAxis;
			leftYAxis = Math.abs(controllers[i].getRawAxis(1)) > Math.abs(leftYAxis) ? controllers[i].getRawAxis(1) : leftYAxis;
			rTrigger = Math.abs(controllers[i].getRawAxis(3)) > Math.abs(rightXAxis) ? controllers[i].getRawAxis(3) : rTrigger;
			rightXAxis = Math.abs(controllers[i].getRawAxis(4)) > Math.abs(rightXAxis) ? controllers[i].getRawAxis(4) : rightXAxis;

			minusButton = minusButton || controllers[i].getRawButton(7);
			plusButton = plusButton || controllers[i].getRawButton(8);
		}

		// Reset the swerve modules when the plus button is pressed.
		if(plusButton)
		{
			this.resetMotors();
		}

		// Reset the gyro when the minus button is pressed.
		if(minusButton)
		{
			this.resetGyro();
			return;
		}
		
		// Vector representation of the left joystick.
		Vec2d movementVec = new Vec2d(-leftXAxis, leftYAxis);

		// Normalize the vector if its length is greater than 1.
		if(movementVec.getLengthSquared() > 1d)
		{
			movementVec = movementVec.normalize();
		}

		// Create a 15% deadzone on the left joystick.
		if(movementVec.getLengthSquared() < 0.0225d)
		{
			movementVec = movementVec.scale(0d);
		}

		// Create a 10% deadzone on the right joystick.
		if(Math.abs(rightXAxis) < 0.25d)
		{
			rightXAxis = 0d;
		}

		// Use the right trigger for speed.
		movementVec = movementVec.scale(this.minMove + (this.maxMove - this.minMove) * rTrigger);

		// Cap the rotational speed.
		rightXAxis = this.maxRot * rightXAxis;

		// Reduce rotational speed if resulting vector's length could be greater than 1.
		if(rightXAxis + movementVec.getLength() > 1d)
		{
			rightXAxis -= movementVec.getLength() + rightXAxis - 1d;
		}

		// Move the drivetrain using the calculated values.
		this.move(movementVec, -rightXAxis);
	}
	
	/**
	 * Set a timer for reseting the motors.
	 */
	public void resetMotors()
	{
		resetTimer = resetTimer <= 0 ? 20 : resetTimer;
	}

	/**
	 * Reset the gyro to 0°.
	 */
	public void resetGyro()
	{
		this.gyro.reset();
	}

	/**
	 * Checks if the robot is resetting or if the gyro is callibrating.
	 * 
	 * @return Whether or not the robot can drive
	 */
	public boolean canDrive()
	{
		return resetTimer <= 0 && !this.gyro.isCalibrating();
	}

	/**
	 * Getter for the gyroscope.
	 * 
	 * @return The gyroscope.
	 */
	public AHRS getGyro()
	{
		return this.gyro;
	}

	/**
	 * Getter for the swerve modules.
	 * 
	 * @return The swerve modules.
	 */
	public List<SwerveModule> getModules()
	{
		return this.modules;
	}

	@Override
	public String toString()
	{
		// The class will be represented as "SwerveDrive[Module1 = {}, Module2 = {}, ...]"
		StringBuilder builder = new StringBuilder("SwerveDrive[");

		for(int i = 0; i < this.modules.size(); i++)
		{
			builder.append("Module" + i + " = {" + this.modules.get(i) + "}, ");
		}

		builder.delete(builder.length() - 2, builder.length());
		builder.append("]");
		return builder.toString();
	}
}
