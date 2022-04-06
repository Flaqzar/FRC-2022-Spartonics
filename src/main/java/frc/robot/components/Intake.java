package frc.robot.components;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Intake implements IControllerMovement
{
    private Spark motor;
    private double suck;
    private double blow;

    public Intake(int intakeMotor, double suckSpeed, double blowSpeed)
    {
        this.motor = new Spark(intakeMotor);
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

        if(rButton)
		{
			this.suck();
		}
		else if(rTrigger)
		{
			this.blow();
		}
		else
		{
			this.hold();
		}
    }

    public void suck()
    {
        this.motor.set(this.suck);
    }

    public void blow()
    {
        this.motor.set(this.blow);
    }

    public void hold()
    {
        this.motor.set(0d);
    }

    public Spark getMotor()
    {
        return this.motor;
    }
}
