package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;

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

    public void turnToAngle(double angle) {

    }
}

