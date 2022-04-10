package frc.robot;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.Callable;

/**
 * Handler for running autonomous code.
 * 
 * @author 2141 Spartonics
 */
public class AutonomousHandler
{
    private final List<Callable<Boolean>> steps;

    private boolean hasFinished;
    private int currentStep;

    /**
     * @param directions the autonomous commands to run.
     */
    @SafeVarargs
    public AutonomousHandler(final Callable<Boolean>... directions)
    {
        this.steps = Collections.unmodifiableList(Arrays.asList(directions));
        this.reset();
    }

    /**
     * Runs the stored autonomous code. 
     * 
     * @throws Exception due to Callables.
     */
    public void run() throws Exception
    {
        if(!this.hasFinished && this.steps.get(currentStep).call().booleanValue())
        {
            this.currentStep++;
            this.hasFinished = currentStep >= this.steps.size();
        }
    }

    /**
     * Resets the handler's variables so that autonomous can be run again.
     */
    public void reset()
    {
        this.hasFinished = false;
        this.currentStep = 0;
    }

    /**
     * Checks if the autonomous code has finished.
     * 
     * @return Whether of not the autonomous code has finished.
     */
    public boolean hasFinished()
    {
        return this.hasFinished;
    }

    /**
     * Getter for the current step being run.
     * 
     * @return The current step.
     */
    public int getCurrentStep()
    {
        return this.currentStep;
    }
}
