package frc.robot.components;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

/**
 * Container for the robot's intake. Wraps a Spark/Neo Motor.
 * 
 * @author 2141 Spartonics
 */
public class Intake implements IControllerMovement, IAutonomous
{
    private final Spark motor;
    private final double suck;
    private final double shoot;

    private long autoEndTime;
    private long autoDuration;

    /**
     * @param intakeMotor the PWM id of the intake motor
     * @param suckSpeed the speed of the motor when sucking in balls
     * @param blowSpeed the speed of the motor when shooting out balls
     */
    public Intake(int intakeMotor, double suckSpeed, double shootSpeed)
    {
        this.motor = new Spark(intakeMotor);
        this.suck = suckSpeed;
        this.shoot = shootSpeed;
        this.autoEndTime = 0l;
        this.autoDuration = 0l;
    }

    @Override
    public void move(XboxController... controllers)
    {
        boolean rButton = false;
		boolean rTrigger = false;

        for(int i = 0; i < controllers.length; i++)
        {
            rButton = rButton || controllers[i].getRightBumper();
            rTrigger = rTrigger || controllers[i].getRawAxis(3) > 0.5d;
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
        this.motor.set(this.suck);
    }

    /**
     * Shoots out the balls.
     */
    public void shoot()
    {
        this.motor.set(this.shoot);
    }

    /**
     * Holds the balls.
     */
    public void hold()
    {
        this.motor.set(0d);
    }

    /**
     * Getter for the intake's motor.
     * 
     * @return The motor used on the intake.
     */
    public Spark getMotor()
    {
        return this.motor;
    }
}
