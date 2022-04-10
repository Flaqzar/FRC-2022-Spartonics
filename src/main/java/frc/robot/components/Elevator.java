package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Container for the robot's elevator.
 * 
 * @author 2141 Spartonics
 */
public class Elevator implements IControllerMovement, IAutonomous
{
    private final WPI_TalonFX motor;
    private final DigitalInput upperSwitch;
    private final DigitalInput lowerSwitch;
    
    /**
     * @param MotorID the CAN id of the motor
     * @param upperLimitSwitchID the PWM id of the upper limit switch
     * @param lowerLimitSwitchID the PWM id of the lower limit switch
     */
    public Elevator(int motorID, int upperLimitSwitchID, int lowerLimitSwitchID)
    {
        this.motor = new WPI_TalonFX(motorID);
        this.upperSwitch = new DigitalInput (upperLimitSwitchID);
        this.lowerSwitch = new DigitalInput (lowerLimitSwitchID);

        this.motor.configFactoryDefault();
    }

    @Override
    public void move(XboxController... controllers)
    {
        boolean lButton = false;
		boolean lTrigger = false;

        int dPad = -1;

        for(int i = 0; i < controllers.length; i++)
        {
            lButton = lButton || controllers[i].getLeftBumper();
            lTrigger = lTrigger || controllers[i].getRawAxis(2) > 0.5d;
            dPad = dPad == -1 ? controllers[i].getPOV() : dPad;
        }

        if(dPad == 315 || dPad == 0 || dPad == 45)
        {
            dPad = -1;
        }
        else if(dPad == 225 || dPad == 180 || dPad == 135)
        {
            dPad = 1;
        }
        else
        {
            dPad = 0;
        }

        if(dPad != 0)
        {
            this.motor.set(0.2d * dPad);
        }
        else if(lTrigger)
		{
			this.goUp();
		}
		else if(lButton)
		{
			this.goDown();
		}
		else
		{
			this.hold();
		}
    }

    /**
     * Moves the elevator for a specified amount of time. The passed in arguments are 
     * direction (1 = raise, -1 = lower, 0 = hold) and duration (in milliseconds or -1 
     * to move to the top/bottom).
     * 
     * @param args direction, duration
     */
    @Override
    public boolean runAuto(double... args)
    {
        return true;
    }

    @Override
    public void startAuto(double... args)
    {
        
    }

    @Override
    public void endAuto(double... args)
    {
        
    }

    /**
     * Raise the elevator.
     */
    public void goUp()
    {
        this.motor.set(1d);
    }

    /**
     * Lower the elevator.
     */
    public void goDown()
    {
        this.motor.set(-1d);
    }

    /**
     * Stop the elevator.
     */
    public void hold()
    {
        this.motor.set(0d);
    }

    /**
     * Getter for the motor.
     * 
     * @return The upward pulling motor
     */
    public WPI_TalonFX getUpMotor()
    {
        return this.motor;
    }

    /**
     * Getter for the limit switch at the top of the elevator.
     * 
     * @return The upper limit switch.
     */
    public DigitalInput getUpperSwitch()
    {
        return this.upperSwitch;
    }

    /**
     * Getter for the limit switch at the bottom of the elevator.
     * 
     * @return The lower limit switch.
     */
    public DigitalInput getLowerSwitch()
    {
        return this.lowerSwitch;
    }
}
