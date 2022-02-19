package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// the axis is used for controlling triggers and joysticks
import edu.wpi.first.wpilibj.XboxController;

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
	/** Main controller */
	private final XboxController controller = new XboxController(0);

	private static boolean bButtonPressed = false;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any
	 * initialization code.
	 */
	@Override
	public void robotInit()
	{
		// ? What does this do?
		this.m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
		this.m_chooser.addOption(kCustomAuto, kCustomAuto);
		SmartDashboard.putData("Auto choices", this.m_chooser);
	}

	@Override
	public void disabledExit()
	{
		this.module1.init();
		this.module2.init();
		this.module3.init();
		this.module4.init();
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for
	 * items like
	 * diagnostics that you want ran during disabled, autonomous, teleoperated and
	 * test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic()
	{
		double leftXAxis = this.controller.getRawAxis(0);
		double leftYAxis = this.controller.getRawAxis(1);

		double Ltrigger = this.controller.getRawAxis(2);
		double Rtrigger = this.controller.getRawAxis(3);

		boolean bButton = this.controller.getBButton();

		if(!bButtonPressed && bButton)
		{
			this.module1.init();
			this.module2.init();
			this.module3.init();
			this.module4.init();
		}

		bButtonPressed = bButton;

		// Creates a deadzone of 10%
		if (leftXAxis * leftXAxis + leftYAxis * leftYAxis > 0.25d)
		{
			this.module1.setAngle(SwerveModule.convertJoystickToAngle(leftXAxis, leftYAxis));
			this.module2.setAngle(SwerveModule.convertJoystickToAngle(leftXAxis, leftYAxis));
			this.module3.setAngle(SwerveModule.convertJoystickToAngle(leftXAxis, leftYAxis));
			this.module4.setAngle(SwerveModule.convertJoystickToAngle(leftXAxis, leftYAxis));
		}

		double speedval = (Ltrigger + Rtrigger) / 2d;

		this.module1.setSpeed(speedval);
		this.module2.setSpeed(speedval);
		this.module3.setSpeed(-speedval);
		this.module4.setSpeed(-speedval);
	}
}
