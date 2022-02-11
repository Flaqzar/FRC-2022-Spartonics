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
	private String m_autoSelected;
	private final SendableChooser<String> m_chooser = new SendableChooser<>();

	//TODO fix CAN bus id's
	SwerveModule module1 = new SwerveModule(2,1);
	SwerveModule module2 = new SwerveModule(4,3);
	SwerveModule module3 = new SwerveModule(6,5);
	SwerveModule module4 = new SwerveModule(8,7);
	/** Main controller */
	private final XboxController controller = new XboxController(0);

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
		
		this.module1.init();
		this.module2.init();
		this.module3.init();
		this.module4.init();
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

		double rightXAxis = this.controller.getRawAxis(4);
		double rightYAxis = this.controller.getRawAxis(5);

		// Creates a deadzone of 10%
		if (leftXAxis * leftXAxis + leftYAxis * leftYAxis > 0.25d)
		{
			this.module1.setAngle(SwerveModule.convertJoystickToAngle(leftXAxis, leftYAxis));
			this.module2.setAngle(SwerveModule.convertJoystickToAngle(leftXAxis, leftYAxis));
			this.module3.setAngle(SwerveModule.convertJoystickToAngle(leftXAxis, leftYAxis));
			this.module4.setAngle(SwerveModule.convertJoystickToAngle(leftXAxis, leftYAxis));
		}

		// Creates a deadzone of 10%
		if (rightXAxis * rightXAxis + rightYAxis * rightYAxis > 0.25d)
		{
		}
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different
	 * autonomous modes using the dashboard. The sendable chooser code works with
	 * the Java
	 * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
	 * chooser code and
	 * uncomment the getString line to get the auto name from the text box below the
	 * Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure
	 * below with additional strings. If using the SendableChooser make sure to add
	 * them to the
	 * chooser code above as well.
	 */
	@Override
	public void autonomousInit()
	{
		this.m_autoSelected = this.m_chooser.getSelected();
		// this.m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
		System.out.println("Auto selected: " + this.m_autoSelected);
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic()
	{
		switch (this.m_autoSelected)
		{
			case kCustomAuto:
				// Put custom auto code here
				break;
			case kDefaultAuto:
			default:
				// Put default auto code here
				break;
		}
	}

	/** This function is called once when teleop is enabled. */
	@Override
	public void teleopInit() {}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {}

	/** This function is called once when the robot is disabled. */
	@Override
	public void disabledInit() {}

	/** This function is called periodically when disabled. */
	@Override
	public void disabledPeriodic() {}

	/** This function is called once when test mode is enabled. */
	@Override
	public void testInit() {}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {}
}
