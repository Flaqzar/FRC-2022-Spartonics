package frc.robot.components;

import edu.wpi.first.wpilibj.PneumaticsControlModule;

public class Pneumatics
{
    private final PneumaticsControlModule pcm;

    public Pneumatics()
    {
        this.pcm = new PneumaticsControlModule();
        
    }

    public void onUpdate()
    {
    }

    public PneumaticsControlModule getPCM()
    {
        return this.pcm;
    }
}
