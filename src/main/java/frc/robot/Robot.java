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

	private static final Compressor COMPRESSOR = new Compressor(PneumaticsModuleType.CTREPCM);
	private static final Solenoid ARM_PULL_SOLENOID = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
	private static final Solenoid ARM_PUSH_SOLENOID = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
	private static final Solenoid HOOK_PULL_SOLENOID = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
	private static final Solenoid HOOK_PUSH_SOLENOID = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
	private static final PneumaticsControlModule REGULATOR = new PneumaticsControlModule();

	/** Main controller */
	private static final XboxController CONTROLLER = new XboxController(0);

	private boolean plusButtonPressed = false;

	private DriveCoordinator drive = new DriveCoordinator();

	public static final Spark INTAKE_MOTOR = new Spark(9);





	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */

	 // I have worked very hard on my essay
	@Override
	public void robotInit()
	{
		drive.GYRO.calibrate();
		this.m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
		this.m_chooser.addOption(kCustomAuto, kCustomAuto);
		SmartDashboard.putData("Auto choices", this.m_chooser);

		// Initializes the swerve modules.
		drive.resetTimer = 10;
	}

	@Override
	public void disabledExit()
	{
		// Resets the swerve module rotation to zero.
		drive.resetTimer = 20;
	}
	/// handle dashboard stuff
	public void handleDash()  
	{
		SmartDashboard.putNumber("Gryo", drive.GYRO.getYaw());
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
			drive.resetTimer = 20;
		}

		// Resets the Gyro's yaw to zero when the minus button is pressed.
		if(minusButton)
		{
			drive.GYRO.reset();
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


		drive.drive(leftXAxis, leftYAxis, rightXAxis, rTrigger);		
	}
}
