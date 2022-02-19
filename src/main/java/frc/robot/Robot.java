package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// the axis is used for controlling triggers and joysticks
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private final SendableChooser<String> m_chooser = new SendableChooser<>();

	// 94.833984375 47.8125 273.33984375 296.54296875
	SwerveModule module1 = new SwerveModule(2, 1, 11, -94.833984375d);
	SwerveModule module2 = new SwerveModule(4, 3, 12, -47.8125d);
	SwerveModule module3 = new SwerveModule(6, 5, 13, -273.33984375d);
	SwerveModule module4 = new SwerveModule(8, 7, 14, -296.54296875d);

	Spark intakeMotor = new Spark(9);

	/** Main controller */
	private final XboxController controller = new XboxController(0);

	private static boolean plusButtonPressed = false;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any nitialization code.
	 */
	@Override
	public void robotInit()
	{
		// Idk what this does.
		this.m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
		this.m_chooser.addOption(kCustomAuto, kCustomAuto);
		SmartDashboard.putData("Auto choices", this.m_chooser);

		// Initializes the swerve modules.
		this.module1.init();
		this.module2.init();
		this.module3.init();
		this.module4.init();
	}

	@Override
	public void disabledExit()
	{
		// Resets the swerve module rotation to zero.
		this.module1.reset();
		this.module2.reset();
		this.module3.reset();
		this.module4.reset();
	}

	@Override
	public void robotPeriodic()
	{
		double leftXAxis = this.controller.getRawAxis(0);
		double leftYAxis = this.controller.getRawAxis(1);
		double rightTrigger = this.controller.getRawAxis(3);

		boolean plusButton = this.controller.getRawButton(8);
		boolean bButton = this.controller.getBButton();

		// Resets the swerve module rotation to zero when the plus button is pressed.
		if(!plusButtonPressed && plusButton)
		{
			this.module1.reset();
			this.module2.reset();
			this.module3.reset();
			this.module4.reset();
		}

		plusButtonPressed = plusButton;

		// Sets the intake motor's speed.		
		if(bButton)
		{
			intakeMotor.set(0.55d);
		}
		else
		{
			intakeMotor.set(0d);
		}

		// Creates a deadzone of 10%.
		if (leftXAxis * leftXAxis + leftYAxis * leftYAxis > 0.25d)
		{
			// Sets the rotation of the swerve modules to the rotaiton of the joystick
			this.module1.setAngle(-SwerveModule.convertJoystickToAngle(leftXAxis, leftYAxis));
			this.module2.setAngle(-SwerveModule.convertJoystickToAngle(leftXAxis, leftYAxis));
			this.module3.setAngle(-SwerveModule.convertJoystickToAngle(leftXAxis, leftYAxis));
			this.module4.setAngle(-SwerveModule.convertJoystickToAngle(leftXAxis, leftYAxis));
		}

		// Sets the speed of the drive motors
		this.module1.setSpeed(rightTrigger);
		this.module2.setSpeed(rightTrigger);
		this.module3.setSpeed(-rightTrigger);
		this.module4.setSpeed(-rightTrigger);
	}
}
