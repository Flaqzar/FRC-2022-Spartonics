package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.components.Bertha;
import frc.robot.components.SwerveDrive;
import frc.robot.components.SwerveModule;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot
{
	/** The main drivetrain of the robot. */
	private static final SwerveDrive DRIVETRAIN = new SwerveDrive(0.3d, 0.8d, 
		new AHRS(SPI.Port.kMXP),
		new SwerveModule(2, 1, 11, -3d * Math.PI / 4d, -274.482421875d),
		new SwerveModule(4, 3, 12, -Math.PI / 4d, -228.33984375d),
		new SwerveModule(6, 5, 13, Math.PI / 4d, -272.900390625d),
		new SwerveModule(8, 7, 14, 3d * Math.PI / 4d, -299.091796875d));

	//private static final Intake INTAKE = new Intake(0, 9, 0.5d, 1d);
	//private static final Elevator ELEVATOR = new Elevator(10, 1, 2);
	/** The giant piston used for the climber */
	private static final Bertha BIG_BERTHA = new Bertha(0, 1);
	/** Slot 0 controller. */
	private static final XboxController PRIMARY_CONTROLLER = new XboxController(0);
	/** Slot 1 controller. */
	private static final XboxController SECONDARY_CONTROLLER = new XboxController(1);
	/** The camera. */
	private static UsbCamera camera;

	private static final AutonomousHandler AUTO_HANDLER = new AutonomousHandler(
		() -> DRIVETRAIN.runAuto(0d, -2d, 0d)
	);

	@Override
	public void robotInit()
	{
		// Start the camera with 320p and 20fps.
		camera = CameraServer.startAutomaticCapture();
		camera.setResolution(320, 240);
		camera.setFPS(20);

		// Calibrate the drivetrain.
		DRIVETRAIN.getGyro().calibrate();
		DRIVETRAIN.resetMotors();
	}

	@Override
	public void disabledExit()
	{
		// Reset the drivetrain.
		DRIVETRAIN.resetMotors();
		// Pull down the piston.
		BIG_BERTHA.pull();
	}

	@Override
	public void autonomousInit()
	{
		AUTO_HANDLER.reset();
		DRIVETRAIN.getGyro().calibrate();
	}

	@Override
	public void autonomousPeriodic()
	{
		// Prevent the robot from doing anything until the gyro has finished calibrating.
		if(DRIVETRAIN.getGyro().isCalibrating()) return;

		try
		{
			AUTO_HANDLER.run();
		}
		catch(Exception e)
		{
			System.out.println("Something has errored in the autonomous!");
			System.out.println(e);
		}
	}

	@Override
	public void teleopPeriodic()
	{
		//ELEVATOR.move(PRIMARY_CONTROLLER, SECONDARY_CONTROLLER);
		//INTAKE.move(PRIMARY_CONTROLLER, SECONDARY_CONTROLLER);
		DRIVETRAIN.move(PRIMARY_CONTROLLER, SECONDARY_CONTROLLER);
		BIG_BERTHA.move(PRIMARY_CONTROLLER, SECONDARY_CONTROLLER);
	}
}
