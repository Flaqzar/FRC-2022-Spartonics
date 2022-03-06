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
	/** Wiimote support */
	public static final boolean IS_WIIMOTE = true;

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
		resetTimer = 20;
	}

	@Override
	public void robotPeriodic()
	{
		double leftXAxis = CONTROLLER.getRawAxis(0);
		double leftYAxis = CONTROLLER.getRawAxis(1);

		double rightXAxis = CONTROLLER.getRawAxis(4);

		boolean minusButton = CONTROLLER.getRawButton(7);
		boolean plusButton = CONTROLLER.getRawButton(8);
		boolean bButton = CONTROLLER.getBButton();
		boolean aButton = CONTROLLER.getAButton();
		double rotMultiplier = Math.round(CONTROLLER.getRawAxis(3) + 1);

		Vec2d joystickVec = new Vec2d(leftXAxis, -leftYAxis).scale(0.5d).rotate(GYRO.getYaw() + 180d, true);
		
		if(IS_WIIMOTE)
		{
			int pov = CONTROLLER.getPOV();
			
			if(pov == 315 || pov == 270 || pov == 226)
			{
				rightXAxis = -1d;
			}
			else if(pov == 45 || pov == 90 || pov == 135)
			{
				rightXAxis = 1d;
			}
			else
			{
				rightXAxis = 0;
			}

			aButton = CONTROLLER.getBButton();
			bButton = CONTROLLER.getYButton();
		}

		System.out.println("a: " + CONTROLLER.getAButton());
		System.out.println("b: " + CONTROLLER.getBButton());
		System.out.println("x: " + CONTROLLER.getXButton());
		System.out.println("y: " + CONTROLLER.getYButton());
		
		// Resets the swerve module rotation to zero when the plus button is pressed.
		if(!plusButtonPressed && plusButton)
		{
			resetTimer = 20;
		}

		if(minusButton)
		{
			//GYRO.calibrate();
			GYRO.reset();
		}

		if(resetTimer > 0)
		{
			resetTimer--;
			MODULE_1.reset();
			MODULE_2.reset();
			MODULE_3.reset();
			MODULE_4.reset();
		}

		plusButtonPressed = plusButton;

		if(bButton)
		{
			INTAKE_MOTOR.set(-0.2d);
		}
		else if(aButton)
		{
			INTAKE_MOTOR.set(0.5d);
		}
		else
		{
			INTAKE_MOTOR.set(0d);
		}

		// Creates a deadzone of 10%.
		if(joystickVec.getLengthSquared() < 0.0225d)
		{
			joystickVec = joystickVec.scale(0d);
		}

		if(resetTimer <= 0)
		{
			double rotSpeed = rightXAxis * rotMultiplier;
			Vec2d rot_1 = new Vec2d(-3d * Math.PI / 4d, -rotSpeed / 2d, false);
			Vec2d rot_2 = new Vec2d(3d * Math.PI / 4d, rotSpeed / 2d, false);
			Vec2d rot_3 = new Vec2d(Math.PI / 4d, rotSpeed / 2d, false);
			Vec2d rot_4 = new Vec2d(-Math.PI / 4d, -rotSpeed / 2d, false);

			System.out.println(rot_1);

			Vec2d s1 = joystickVec.scale(-1d).add(rot_1);
			Vec2d s2 = joystickVec.scale(-1d).add(rot_2);
			Vec2d s3 = joystickVec.add(rot_3);
			Vec2d s4 = joystickVec.add(rot_4);

			System.out.println(s1);
			System.out.println();

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
}
