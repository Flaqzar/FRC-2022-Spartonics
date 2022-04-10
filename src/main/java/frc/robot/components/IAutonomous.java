package frc.robot.components;

/**
 * Interface for running autonomous code.
 * 
 * @author 2141 Spartonics
 */
public interface IAutonomous
{
    /**
     * Runs before the main runAuto() function.
     * 
     * @param args autonomous arguments
     * @return Whether or not the action has finished.
     */
    public void startAuto(double... args);

    /**
     * Run autonomous code.
     * 
     * @param args autonomous arguments
     * @return Whether or not the action has finished.
     */
    public boolean runAuto(double... args);

    /**
     * Runs after the main runAuto() function but before the next command.
     * 
     * @param args autonomous arguments
     * @return Whether or not the action has finished.
     */
    public void endAuto(double... args);
}
