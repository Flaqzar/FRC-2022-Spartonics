package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package
 * after creating this project, you must also update the build.gradle file
 * in the project.
 */
public class Robot extends TimedRobot
{
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private final SendableChooser<String> m_chooser = new SendableChooser<String>();
	private static int resetTimer = 0;

	// Can offsets: 94.833984375, 47.8125, 273.33984375, 296.54296875
	// Initialize all robot related the variables
	private static final SwerveModule MODULE_1 = new SwerveModule(2, 1, 11, -94.833984375d);
	private static final SwerveModule MODULE_2 = new SwerveModule(4, 3, 12, -47.8125d);
	private static final SwerveModule MODULE_3 = new SwerveModule(6, 5, 13, -273.33984375d);
	private static final SwerveModule MODULE_4 = new SwerveModule(8, 7, 14, -296.54296875d);
	private static final Spark INTAKE_MOTOR = new Spark(9);
	private static final AHRS GYRO = new AHRS(SPI.Port.kMXP);

	/** Main controller */
	private static final XboxController CONTROLLER = new XboxController(0);

	private boolean plusButtonPressed = false;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit()
	{
		// Idk what this does.
		this.m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
		this.m_chooser.addOption(kCustomAuto, kCustomAuto);
		SmartDashboard.putData("Auto choices", this.m_chooser);

		// Initializes the swerve modules.
		resetTimer = 10;
	}

	@Override
	public void disabledExit()
	{
		// Resets the swerve module rotation to zero.
		MODULE_1.reset();
		MODULE_2.reset();
		MODULE_3.reset();
		MODULE_4.reset();
	}

	@Override
	public void robotPeriodic()
	{
		double leftXAxis = CONTROLLER.getRawAxis(0);
		double leftYAxis = CONTROLLER.getRawAxis(1);

		double rightXAxis = CONTROLLER.getRawAxis(4);
		double rightYAxis = CONTROLLER.getRawAxis(5);

		boolean plusButton = CONTROLLER.getRawButton(8);
		boolean bButton = CONTROLLER.getBButton();
		boolean aButton = CONTROLLER.getAButton();

		double joystickDistance = Math.min(Math.sqrt(leftXAxis * leftXAxis + leftYAxis * leftYAxis), 1d);

		// Resets the swerve module rotation to zero when the plus button is pressed.
		if(!plusButtonPressed && plusButton)
		{
			resetTimer = 10;
		}

		if(resetTimer > 0)
		{
			resetTimer--;
			MODULE_1.reset();
			MODULE_2.reset();
			MODULE_3.reset();
			MODULE_4.reset();

			if(resetTimer == 0)
			{
				GYRO.calibrate();
			}
		}

		plusButtonPressed = plusButton;

		if (bButton) {
			INTAKE_MOTOR.set(0.25d);
		} else {
			INTAKE_MOTOR.set(aButton ? -0.25d : 0d);
		}

		// Creates a deadzone of 10%.
		if (joystickDistance > 0.1d || rightXAxis * rightXAxis + rightYAxis * rightYAxis > 0.25d)
		{
			double gyroRotation = GYRO.getYaw() * Constants.TWO_PI / 360d;
			double rotateAngle = rightXAxis * Constants.PI_OVER_TWO / 2d;
			System.out.println(rotateAngle);
			// Sets the rotation of the swerve modules to the rotaiton of the joystick
			MODULE_1.setAngle(-SwerveModule.convertJoystickToAngle(leftXAxis, leftYAxis) + Math.PI + rotateAngle + Constants.PI_OVER_TWO * 0d);
			MODULE_2.setAngle(-SwerveModule.convertJoystickToAngle(leftXAxis, leftYAxis) + Math.PI + rotateAngle + Constants.PI_OVER_TWO * 1d);
			MODULE_3.setAngle(-SwerveModule.convertJoystickToAngle(leftXAxis, leftYAxis) + Math.PI + rotateAngle + Constants.PI_OVER_TWO * 2d);
			MODULE_4.setAngle(-SwerveModule.convertJoystickToAngle(leftXAxis, leftYAxis) + Math.PI + rotateAngle + Constants.PI_OVER_TWO * 3d);

			double speed = joystickDistance * (CONTROLLER.getRightTriggerAxis() > 0.5d ? 1d : 0.5d);

			// Sets the speed of the drive motors
			MODULE_1.setSpeed(speed);
			MODULE_2.setSpeed(speed);
			MODULE_3.setSpeed(-speed);
			MODULE_4.setSpeed(-speed);
		}
		else
		{
			// Sets the speed of the drive motors
			MODULE_1.setSpeed(0d);
			MODULE_2.setSpeed(0d);
			MODULE_3.setSpeed(0d);
			MODULE_4.setSpeed(0d);
		}
	}
}
