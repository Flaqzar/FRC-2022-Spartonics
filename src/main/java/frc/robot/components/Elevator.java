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
    private final WPI_TalonFX upMotor;
    private final WPI_TalonFX downMotor;
    private final DigitalInput upperSwitch;
    private final DigitalInput lowerSwitch;
    
    /**
     * @param upMotorID the CAN id of the upward pulling motor
     * @param downMotorID the CAN id of the downward pullling motor
     * @param upperLimitSwitchID the PWM id of the upper limit switch
     * @param lowerLimitSwitchID the PWM id of the lower limit switch
     */
    public Elevator(int upMotorID, int downMotorID, int upperLimitSwitchID, int lowerLimitSwitchID)
    {
        this.upMotor = new WPI_TalonFX(upMotorID);
        this.downMotor = new WPI_TalonFX(downMotorID);
        this.upperSwitch = new DigitalInput (upperLimitSwitchID);
        this.lowerSwitch = new DigitalInput (lowerLimitSwitchID);

        this.upMotor.configFactoryDefault();
        this.downMotor.configFactoryDefault();
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
            this.upMotor.set(0.2d * dPad);
        }
        else if(lButton)
		{
			this.goUp();
		}
		else if(lTrigger)
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

    /**
     * Raise the elevator.
     */
    public void goUp()
    {
        this.upMotor.set(1d);
        this.downMotor.set(-1d);
    }

    /**
     * Lower the elevator.
     */
    public void goDown()
    {
        this.upMotor.set(-1d);
        this.downMotor.set(1d);
    }

    /**
     * Stop the elevator.
     */
    public void hold()
    {
        this.upMotor.set(0d);
        this.downMotor.set(0d);
    }

    /**
     * Getter for the upward pulling motor.
     * 
     * @return The upward pulling motor
     */
    public WPI_TalonFX getUpMotor()
    {
        return this.upMotor;
    }

    /**
     * Getter for the downward pulling motor.
     * 
     * @return The downward pulling motor
     */
    public WPI_TalonFX getDownMotor()
    {
        return this.downMotor;
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
