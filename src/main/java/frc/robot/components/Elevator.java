package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Container for the robot's elevator.
 */
public class Elevator implements IControllerMovement
{
    private final WPI_TalonFX upMotor;
    private final WPI_TalonFX downMotor;
    private final DigitalInput  upperSwitch;
    private final DigitalInput  lowerSwitch;
    
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

        for(int i = 0; i < controllers.length; i++)
        {
            lButton = lButton || controllers[i].getLeftBumper();
            lTrigger = lTrigger || controllers[i].getRawAxis(2) > 0.5d;
        }

        if(lTrigger && !this.upperSwitch.get())
		{
			this.goUp();
		}
		else if(lButton && !this.lowerSwitch.get())
		{
			this.goDown();
		}
		else
		{
			this.hold();
		}
    }

    /**
     * Raise the elevator.
     */
    public void goUp()
    {
        this.upMotor.set(1d);
        this.downMotor.set(1d);
    }

    /**
     * Lower the elevator.
     */
    public void goDown()
    {
        this.upMotor.set(-1d);
        this.downMotor.set(-1d);
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
}
