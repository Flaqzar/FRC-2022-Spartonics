package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.components.SwerveDrive;
import frc.robot.components.SwerveModule;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The main robot class where everything is run.
 * 
 * @author 2141 Spartonics
 */
public class Robot extends TimedRobot
{
	/** The main drivetrain of the robot. */
	private static final SwerveDrive DRIVETRAIN = new SwerveDrive(0.3d, 0.8d, 0.5d, 
		new AHRS(SPI.Port.kMXP),
		new SwerveModule(2, 1, 11, -3d * Math.PI / 4d, -274.482421875d),
		new SwerveModule(4, 3, 12, -Math.PI / 4d, -228.33984375d),
		new SwerveModule(6, 5, 13, Math.PI / 4d, -272.900390625d),
		new SwerveModule(8, 7, 14, 3d * Math.PI / 4d, -299.091796875d));

	/** Slot 0 controller. */
	private static final XboxController PRIMARY_CONTROLLER = new XboxController(0);

	@Override
	public void robotInit()
	{
		// Reset the drivetrain and cailbrate the gyroscope.
		DRIVETRAIN.getGyro().calibrate();
		DRIVETRAIN.resetMotors();
	}

	@Override
	public void disabledExit()
	{
		// Reset the drivetrain.
		DRIVETRAIN.resetMotors();
	}

	@Override
	public void teleopPeriodic()
	{
		// Move the drivetrian.
		DRIVETRAIN.move(PRIMARY_CONTROLLER);
	}
}
