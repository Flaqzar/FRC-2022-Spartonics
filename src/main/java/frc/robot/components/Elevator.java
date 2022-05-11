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
	/** Main motor for raising and lowering the elevator. */
	private final WPI_TalonFX motor;
	/** Limit switch at the top of the elevator. */
	private final DigitalInput upperSwitch;
	/** Limit switch at the bottom of the elevator. */
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
		// Button values.
		boolean lButton = false;
		boolean lTrigger = false;

		// D-pad value.
		int dPad = -1;

		// Iterate through the controllers to get their combined inputs.
		for(int i = 0; i < controllers.length; i++)
		{
			lButton = lButton || controllers[i].getLeftBumper();
			lTrigger = lTrigger || controllers[i].getRawAxis(2) > 0.5d;
			dPad = dPad == -1 ? controllers[i].getPOV() : dPad;
		}

		// Set the d-pad value to -1 when the bottom is pushed.
		if(dPad == 315 || dPad == 0 || dPad == 45)
		{
			dPad = -1;
		}
		// Set the d-pad value to 1 when the top is pushed.
		else if(dPad == 225 || dPad == 180 || dPad == 135)
		{
			dPad = 1;
		}
		// Set the d-pad value to 0 otherwise.
		else
		{
			dPad = 0;
		}

		// Fine tune the motor if using the d-pad.
		if(dPad != 0)
		{
			this.motor.set(0.2d * dPad);
		}
		// Move the elevator down if the left trigger is pressed.
		else if(lTrigger)
		{
			this.goDown();
		}
		// Move the elevator up if the left bumper is pressed.
		else if(lButton)
		{
			this.goUp();
		}
		// Stop the elevator otherwise.
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
