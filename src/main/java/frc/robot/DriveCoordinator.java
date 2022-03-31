package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;


public class DriveCoordinator {

    public static int resetTimer = 0;

	// Can offsets: 94.833984375, 47.8125, 273.33984375, 296.54296875
	// Initialize all robot related the variables
    public static final SwerveModule MODULE_1 = new SwerveModule(2, 1, 11, 0d);
    public static final SwerveModule MODULE_2 = new SwerveModule(4, 3, 12, 0d);
    public static final SwerveModule MODULE_3 = new SwerveModule(6, 5, 13, 0d);
    public static final SwerveModule MODULE_4 = new SwerveModule(8, 7, 14, 0d);
    public static final AHRS GYRO = new AHRS(SPI.Port.kMXP);

    public void drive(double leftXAxis, double leftYAxis, double rightXAxis, boolean rTrigger) {
        Vec2d movementVec = new Vec2d(-leftXAxis, leftYAxis).rotate(GYRO.getYaw() + 180d, true).scale(rTrigger ? 1d : 0.33d);

        // Resets the swerve modules' rotations to zero.
        if(resetTimer > 0)
		{
			resetTimer--;
			MODULE_1.reset();
			MODULE_2.reset();
			MODULE_3.reset();
			MODULE_4.reset();
		}
        // Creates a deadzone of 15% for the movement vector.
		if(movementVec.getLengthSquared() < 0.0225d)
		{
			movementVec = movementVec.scale(0d);
		}

        		// Creates a deadzone of 10% for the rotation.
		if(Math.abs(rightXAxis) < 0.1d)
		{
			rightXAxis = 0d;
		}

        // Only run while the robot is not calibrating.
		if(resetTimer <= 0 && !GYRO.isCalibrating())
		{
			// The steering motors' speed.
			double rotSpeed = rTrigger ? 0d : rightXAxis * rightXAxis * rightXAxis / 4d;
			
			// Creates rotational vectors for each swerve module.
			Vec2d rotVec1 = new Vec2d(-3d * Math.PI / 4d, -rotSpeed, false);
			Vec2d rotVec2 = new Vec2d(3d * Math.PI / 4d, rotSpeed, false);
			Vec2d rotVec3 = new Vec2d(Math.PI / 4d, rotSpeed, false);
			Vec2d rotVec4 = new Vec2d(-Math.PI / 4d, -rotSpeed, false);
			
			// Adds the rotational vectors to the movement vectors.
			Vec2d s1 = movementVec.add(rotVec1);
			Vec2d s2 = movementVec.add(rotVec2);
			Vec2d s3 = movementVec.add(rotVec3);
			Vec2d s4 = movementVec.add(rotVec4);

			// Sets the angle of the steering motors.
			MODULE_1.setAngle(s1);
			MODULE_2.setAngle(s2);
			MODULE_3.setAngle(s3);
			MODULE_4.setAngle(s4);

			// Sets the speed of the drive motors
			MODULE_1.setSpeed(s1.getLength());
			MODULE_2.setSpeed(s2.getLength());
			MODULE_3.setSpeed(s3.getLength());
			MODULE_4.setSpeed(s4.getLength());
			
		}

    }
	/// drives a distance (in inches) at a given angle
	public void driveDistanceAtAngle(double dist, double angle) {
		double distance = dist * Constants.TICKS_PER_INCH;
		turnToAngle(angle);
		resetDriveEncoders();

	

		MODULE_1.drivingFalcon.set(ControlMode.MotionMagic, distance);
		MODULE_2.drivingFalcon.set(ControlMode.MotionMagic, distance);
		MODULE_3.drivingFalcon.set(ControlMode.MotionMagic, distance);
		MODULE_4.drivingFalcon.set(ControlMode.MotionMagic, distance);

	}


	/// resets the encoders for each swerve module's drive motor
	public void resetDriveEncoders() {
		MODULE_1.resetDriveEncoder();
		MODULE_2.resetDriveEncoder();
		MODULE_3.resetDriveEncoder();
		MODULE_4.resetDriveEncoder();
	}

	/// Resets the swerve modules' rotations to zero.
	public void reset()
	{
		resetTimer = 10;
	}

    public void turnToAngle(double angle) {
		MODULE_1.setAngle(new Vec2d(angle, 0d, false));
		MODULE_2.setAngle(new Vec2d(angle, 0d, false));
		MODULE_3.setAngle(new Vec2d(angle, 0d, false));
		MODULE_4.setAngle(new Vec2d(angle, 0d, false));
    }


}

