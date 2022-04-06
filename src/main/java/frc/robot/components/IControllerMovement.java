package frc.robot.components;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Interface for controlling robot part movemenbt via XBox controllers.
 */
public interface IControllerMovement
{
    /**
     * Moves the robot part based on controller inputs.
     * 
     * @param controllers the controllers that can move this part.
     */
    public void move(XboxController... controllers);
}
