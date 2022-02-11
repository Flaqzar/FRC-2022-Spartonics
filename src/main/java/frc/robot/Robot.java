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

	SwerveModule module1 = new SwerveModule(2, 1, 1); // ToDo: set canCoder id to correct values
	SwerveModule module2 = new SwerveModule(4, 3, 2);
	SwerveModule module3 = new SwerveModule(6, 5, 3);
	SwerveModule module4 = new SwerveModule(8, 7, 4);
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

		// Creates a deadzone of 10%
		if (leftXAxis * leftXAxis + leftYAxis * leftYAxis > 0.25d)
		{
			this.module1.setAngle(SwerveModule.convertJoystickToAngle(leftXAxis, leftYAxis));
			this.module2.setAngle(SwerveModule.convertJoystickToAngle(leftXAxis, leftYAxis));
			this.module3.setAngle(SwerveModule.convertJoystickToAngle(leftXAxis, leftYAxis));
			this.module4.setAngle(SwerveModule.convertJoystickToAngle(leftXAxis, leftYAxis));
		}
	}
}
