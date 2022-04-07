package frc.robot.components;

/**
 * Interface for running autonomous code.
 * 
 * @author 2141 Spartonics
 */
public interface IAutonomous
{
    /**
     * Run autonomous code.
     * 
     * @param args autonomous arguments
     * @return Whether or not the action has finished.
     */
    public boolean runAuto(double... args);
}
