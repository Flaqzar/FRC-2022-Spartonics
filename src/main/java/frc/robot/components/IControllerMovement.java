package frc.robot.components;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Interface for controlling robot part movement via XBox controllers.
 * 
 * @author 2141 Spartonics
 */
public interface IControllerMovement
{
	/**
	 * Moves the robot part based on controller inputs.
	 * 
	 * @param controllers the controllers that can move this part
	 */
	public void move(XboxController... controllers);
}
