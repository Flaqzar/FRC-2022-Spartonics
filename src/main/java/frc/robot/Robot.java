package frc.robot;

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


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package
 * after creating this project, you must also update the build.gradle file
 * in the project.
 */
public class Robot extends TimedRobot
{

	PowerDistribution PDB = new PowerDistribution(1, ModuleType.kCTRE);
	/** Wiimote support */
	public static final boolean IS_WIIMOTE = false;

	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private final SendableChooser<String> m_chooser = new SendableChooser<String>();
	private static int resetTimer = 0;

	// Can offsets: 94.833984375, 47.8125, 273.33984375, 296.54296875
	// Initialize all robot related the variables
	private static final SwerveModule MODULE_1 = new SwerveModule(2, 1, 11, 0d);
	private static final SwerveModule MODULE_2 = new SwerveModule(4, 3, 12, 0d);
	private static final SwerveModule MODULE_3 = new SwerveModule(6, 5, 13, 0d);
	private static final SwerveModule MODULE_4 = new SwerveModule(8, 7, 14, 0d);
	private static final Spark INTAKE_MOTOR = new Spark(9);
	private static final AHRS GYRO = new AHRS(SPI.Port.kMXP);
	private static final Compressor COMPRESSOR = new Compressor(PneumaticsModuleType.CTREPCM);
	private static final Solenoid ARM_PULL_SOLENOID = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
	private static final Solenoid ARM_PUSH_SOLENOID = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
	private static final Solenoid HOOK_PULL_SOLENOID = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
	private static final Solenoid HOOK_PUSH_SOLENOID = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
	private static final PneumaticsControlModule REGULATOR = new PneumaticsControlModule();

	/** Main controller */
	private static final XboxController CONTROLLER = new XboxController(0);

	private boolean plusButtonPressed = false;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */

	 // I have worked very hard on my essay
	@Override
	public void robotInit()
	{
		GYRO.calibrate();
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
	/// handle dashboard stuff
	public void handleDash()  
	{
		SmartDashboard.putNumber("Gryo", GYRO.getYaw());
	}

	@Override
	public void robotPeriodic()
	{
		this.handleDash();

		// TODO Re-align the swerve modules.
		//System.out.println("Module 1: " + MODULE_1.getCanCoder().getAbsolutePosition());
		//System.out.println("Module 2: " + MODULE_2.getCanCoder().getAbsolutePosition());
		//System.out.println("Module 3: " + MODULE_3.getCanCoder().getAbsolutePosition());
		//System.out.println("Module 4: " + MODULE_4.getCanCoder().getAbsolutePosition());

		double leftXAxis = CONTROLLER.getRawAxis(0);
		double leftYAxis = CONTROLLER.getRawAxis(1);

		double rightXAxis = CONTROLLER.getRawAxis(4);

		boolean minusButton = CONTROLLER.getRawButton(7);
		boolean plusButton = CONTROLLER.getRawButton(8);
		boolean bButton = CONTROLLER.getBButton();
		boolean aButton = CONTROLLER.getAButton();
		boolean yButton = CONTROLLER.getYButton();
		boolean xButton = CONTROLLER.getXButton();
		boolean rTrigger = CONTROLLER.getRawAxis(3) > 0.5d;

		// The robot's movement vector. Determined by 
		Vec2d movementVec = new Vec2d(-leftXAxis, leftYAxis).rotate(GYRO.getYaw() + 180d, true).scale(rTrigger ? 1d : 0.33d);
		
		// For Fun. Allows a wiimote to control the robot.
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

		// Testing solenoids.
		if(yButton)
		{
			ARM_PUSH_SOLENOID.set(true);
			ARM_PULL_SOLENOID.set(false);
		}
		else
		{
			ARM_PUSH_SOLENOID.set(false);
			ARM_PULL_SOLENOID.set(true);
		}

		if(xButton)
		{
			HOOK_PUSH_SOLENOID.set(true);
			HOOK_PULL_SOLENOID.set(false);
		}
		else
		{
			HOOK_PUSH_SOLENOID.set(false);
			HOOK_PULL_SOLENOID.set(true);
		}
		
		// Resets the swerve module rotation to zero when the plus button is pressed.
		if(!plusButtonPressed && plusButton)
		{
			resetTimer = 20;
		}

		// Resets the Gyro's yaw to zero when the minus button is pressed.
		if(minusButton)
		{
			GYRO.reset();
		}

		// Resets the swerve modules' rotations to zero.
		if(resetTimer > 0)
		{
			resetTimer--;
			MODULE_1.reset();
			MODULE_2.reset();
			MODULE_3.reset();
			MODULE_4.reset();
		}

		// Stores when the plus button is pressed so it only triggers once when pressed.
		plusButtonPressed = plusButton;

		// Sets the intake motor's speed.
		// A button sucks in the balls.
		// B button spits out the balls.
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
}
