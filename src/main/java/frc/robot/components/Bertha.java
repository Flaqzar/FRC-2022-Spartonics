package frc.robot.components;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;

public class Bertha implements IControllerMovement
{
    private final Solenoid push;
    private final Solenoid pull;

    public Bertha(int pushSolenoidID, int pullSolenoidID)
    {
        this.push = new Solenoid(PneumaticsModuleType.CTREPCM, pushSolenoidID);
        this.pull = new Solenoid(PneumaticsModuleType.CTREPCM, pullSolenoidID);
    }

    @Override
    public void move(XboxController... controllers)
    {
        // TODO set controls.
    }
    
    public void push()
    {
        this.push.set(true);
        this.pull.set(false);
    }

    public void pull()
    {
        this.push.set(false);
        this.pull.set(true);
    }
}
