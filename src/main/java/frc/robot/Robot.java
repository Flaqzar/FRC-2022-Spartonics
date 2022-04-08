package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.components.Bertha;
import frc.robot.components.Elevator;
import frc.robot.components.Intake;
import frc.robot.components.SwerveDrive;
import frc.robot.components.SwerveModule;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot
{
    private static final SwerveDrive DRIVETRAIN = new SwerveDrive(
        new AHRS(SPI.Port.kMXP),
        new SwerveModule(2, 1, 11, -3d * Math.PI / 4d, -274.482421875d),
        new SwerveModule(4, 3, 12, -Math.PI / 4d, -228.33984375d),
        new SwerveModule(6, 5, 13, Math.PI / 4d, -272.900390625d),
        new SwerveModule(8, 7, 14, 3d * Math.PI / 4d, -299.091796875d));

	private static final Intake INTAKE = new Intake(0, 0.5d, -0.2d);
    private static final Elevator ELEVATOR = new Elevator(9, 10, 1, 2);
    private static final Bertha BIG_BERTHA = new Bertha(0, 1);
	private static final XboxController PRIMARY_CONTROLLER = new XboxController(0);
    private static final XboxController SECONDARY_CONTROLLER = new XboxController(1);

    private static final AutonomousHandler AUTO_HANDLER = new AutonomousHandler(
        () -> DRIVETRAIN.runAuto(0.25d, 0d, 0d),
        () -> INTAKE.runAuto(1, 3000),
        () -> DRIVETRAIN.runAuto(-1d, 0d, 0d)
    );

	@Override
	public void robotInit()
	{
		DRIVETRAIN.getGyro().calibrate();
		DRIVETRAIN.resetMotors();
	}

	@Override
	public void disabledExit()
	{
		DRIVETRAIN.resetMotors();
	}

    @Override
    public void autonomousInit()
    {
        AUTO_HANDLER.reset();
        DRIVETRAIN.getGyro().resetDisplacement();
        DRIVETRAIN.getGyro().reset();
    }

    @Override
    public void autonomousPeriodic()
    {
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
		ELEVATOR.move(PRIMARY_CONTROLLER, SECONDARY_CONTROLLER);
        INTAKE.move(PRIMARY_CONTROLLER, SECONDARY_CONTROLLER);
        DRIVETRAIN.move(PRIMARY_CONTROLLER, SECONDARY_CONTROLLER);
        BIG_BERTHA.move(PRIMARY_CONTROLLER, SECONDARY_CONTROLLER);
	}
}
