package frc.robot.components;

import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
/**
 * Container for the robot's intake. Wraps a Spark/Neo Motor.
 * 
 * @author 2141 Spartonics
 */
@Deprecated
public class Intake implements IControllerMovement, IAutonomous
{
	private final WPI_TalonFX suckMotor;
	private final WPI_TalonFX stopMotor;


	private final double suck;
	private final double shoot;

	private long autoEndTime;
	private long autoDuration;

	/**
	 * @param intakeSuckMotor the PWM id of the intake sucking motor
	 * @param intakeStopMotor the PWM id of the intake stopping motor
	 * @param suckSpeed the speed of the motor when sucking in balls
	 * @param blowSpeed the speed of the motor when shooting out balls
	 */
	public Intake(int intakeSuckMotor, int intakeStopMotor, double suckSpeed, double shootSpeed)
	{
		this.suckMotor = new WPI_TalonFX(intakeSuckMotor);
		this.stopMotor = new WPI_TalonFX(intakeStopMotor);
		this.suck = suckSpeed;
		this.shoot = shootSpeed;
		this.autoEndTime = 0l;
		this.autoDuration = 0l;

		this.suckMotor.configFactoryDefault();
		this.stopMotor.configFactoryDefault();
	}

	@Override
	public void move(XboxController... controllers)
	{
		boolean rButton = false;
		boolean rTrigger = false;
		boolean xButton = false;
		boolean bButton = false;

		for(int i = 0; i < controllers.length; i++)
		{
			rButton = rButton || controllers[i].getRightBumper();
			rTrigger = rTrigger || controllers[i].getRawAxis(3) > 0.5d;
			xButton = xButton || controllers[i].getXButton();
			bButton = bButton || controllers[i].getBButton();

		}

		if(rTrigger)
		{
			this.suck();
		}
		else if(rButton)
		{
			this.shoot();
		}
		else
		{
			this.hold();
		}

		if (xButton)
		{
			this.stop();
		}
		else if (bButton)
		{
			this.unstop();
		}
	}

	/**
	 * Turns on the intake for a specified amount of time. The passed in arguments are 
	 * direction (1 = shoot, -1 = suck, 0 = hold) and duration (in milliseconds or -1 
	 * to enable until concurrent steps have finished).
	 * 
	 * @param args direction, duration
	 */
	@Override
	public boolean runAuto(double... args)
	{
		if(this.autoDuration != args[1])
		{
			this.autoDuration = (long) args[1];
			this.autoEndTime = this.autoDuration + System.currentTimeMillis();
		}

		switch((int) args[0])
		{
		case -1:
			this.suck();
			break;
		case 1:
			this.shoot();
			break;
		default:
			this.hold();
			break;
		}

		return args[1] == -1 ||  this.autoEndTime <= System.currentTimeMillis();
	}

	/**
	 * Sucks in the balls.
	 */
	public void suck()
	{
		this.suckMotor.set(this.suck);
	}

	/**
	 * Shoots out the balls.
	 */
	public void shoot()
	{
		this.suckMotor.set(this.shoot);
	}

	/**
	 * Holds the balls.
	 */
	public void hold()
	{
		this.suckMotor.set(0d);
	}

	/**
	 * Stops the balls from being spitted out.
	 */
	public void stop()
	{
		this.stopMotor.set(-1d);
	}

	/**
	 * Unstops the balls.
	 */
	public void unstop()
	{
		this.stopMotor.set(1d);
	}

	/**
	 * Getter for the intake's sucking motor.
	 * 
	 * @return The motor used on the intake.
	 */
	public WPI_TalonFX getSuckingMotor()
	{
		return this.suckMotor;
	}

	/**
	 * Getter for the intake's stopper motor.
	 * 
	 * @return The motor used on the intake.
	 */
	public WPI_TalonFX getStoppingMotor()
	{
		return this.suckMotor;
	}
}
