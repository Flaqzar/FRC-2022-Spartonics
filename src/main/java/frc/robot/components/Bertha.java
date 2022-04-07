package frc.robot.components;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Container for Big Bertha, the massive piston.
 * 
 * @author 2141 Spartonics
 */
public class Bertha implements IControllerMovement, IAutonomous
{
    private final Solenoid push;
    private final Solenoid pull;

    /**
     * @param pushSolenoidID the PCM id of the piston pushing solenoid
     * @param pullSolenoidID the PCM id of the piston pulling solenoid
     */
    public Bertha(int pushSolenoidID, int pullSolenoidID)
    {
        this.push = new Solenoid(PneumaticsModuleType.CTREPCM, pushSolenoidID);
        this.pull = new Solenoid(PneumaticsModuleType.CTREPCM, pullSolenoidID);
    }

    @Override
    public void move(XboxController... controllers)
    {
        boolean yButton = false;
        boolean aButton = false;

        for(int i = 0; i < controllers.length; i++)
        {
            yButton = yButton || controllers[i].getYButton();
            aButton = aButton || controllers[i].getAButton();
        }

        if(yButton)
        {
            this.push();
        }
        else if(aButton)
        {
            this.pull();
        }
    }

    /**
     * Raises/Lowers Big Bertha. The passed in argument is direction (-1 to lower, 1 to raise)
     * 
     * @param args direciton
     */
    @Override
    public boolean runAuto(double... args)
    {
        return true;
    }
    
    /**
     * Push Big Bertha up.
     */
    public void push()
    {
        this.push.set(true);
        this.pull.set(false);
    }

    /**
     * Pull Big Bertha down.
     */
    public void pull()
    {
        this.push.set(false);
        this.pull.set(true);
    }

    /**
     * Getter for the pushing solenoid.
     * 
     * @return The push solenoid.
     */
    public Solenoid getPushSolenoid()
    {
        return this.push;
    }

    /**
     * Getter for the pulling solenoid.
     * 
     * @return The pull solenoid.
     */
    public Solenoid getPullSolenoid()
    {
        return this.pull;
    }
}
