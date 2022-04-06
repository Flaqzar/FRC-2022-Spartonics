package frc.robot.components;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.math.Constants;
import frc.robot.math.Vec2d;

/**
 * Represents a swerve drivetrain.
 */
public class SwerveDrive implements IControllerMovement
{
    private static int resetTimer = 0;
    private final AHRS gyro;
    private final List<SwerveModule> modules;
    
    /**
     * Creates a swerve drive handler.
     * 
     * @param gyroscope the swerve drive's gyroscope
     * @param swerveModules the swerve drive's wheel modules
     */
    public SwerveDrive(AHRS gyroscope, SwerveModule... swerveModules)
    {
        this.gyro = gyroscope;
        this.modules = Collections.unmodifiableList(Arrays.asList(swerveModules));
    }

    /**
     * Move's the swerve drive. The direction vector is automatically adjusted by the 
     * gyroscope. Be wary that the maximum speed of the motors is 1. Both the magnitude 
     * of the directional vector and the rotational speed added together must be less 
     * than or equal to 1.
     * 
     * @param directionVec the direction and speed to travel (magnitude from -1 to 1)
     * @param rotationalSpeed how fast the robot rotates (from -1 to 1)
     */
    public void move(Vec2d directionVec, double rotationalSpeed)
    {
        if(this.canDrive())
        {
            Vec2d driveVec = directionVec.rotate(this.gyro.getYaw() + 180d, true);
        
            this.modules.forEach(m ->
            {
                Vec2d rotVec = new Vec2d(m.getRotationDirection(), rotationalSpeed, false);
                Vec2d finalVec = driveVec.add(rotVec);
                m.setAngle(finalVec);
                m.setSpeed(finalVec.getLength());
            });
        }
        else
        {
            this.modules.forEach(m -> m.reset());
            resetTimer--;
        }
    }

    @Override
    public void move(XboxController... controllers)
    {
        double leftXAxis = 0d;
		double leftYAxis = 0d;
		double rightXAxis = 0d;

        boolean minusButton = false;
		boolean plusButton = false;

        for(int i = 0; i < controllers.length; i++)
        {
            leftXAxis =  Math.abs(controllers[i].getRawAxis(0)) > Math.abs(leftXAxis) ? controllers[i].getRawAxis(0) : leftXAxis;
            leftYAxis =  Math.abs(controllers[i].getRawAxis(1)) > Math.abs(leftYAxis) ? controllers[i].getRawAxis(1) : leftYAxis;
            rightXAxis =  Math.abs(controllers[i].getRawAxis(4)) > Math.abs(rightXAxis) ? controllers[i].getRawAxis(4) : leftXAxis;
            minusButton = minusButton || controllers[i].getRawButton(7);
            plusButton = plusButton || controllers[i].getRawButton(8);
        }

        if(plusButton)
		{
			this.resetMotors();
		}

		if(minusButton)
		{
			this.resetGyro();
            return;
		}
        
        Vec2d movementVec = new Vec2d(-leftXAxis, leftYAxis).scale(0.33d);

        if(movementVec.getLengthSquared() < 0.0225d)
		{
			movementVec = movementVec.scale(0d);
		}

        if(Math.abs(rightXAxis) < 0.1d)
		{
			rightXAxis = 0d;
		}

        this.move(movementVec, rightXAxis);
    }

    /**
     * Sets a timer that resets the motors.
     */
    public void resetMotors()
    {
        resetTimer = resetTimer <= 0 ? 20 : resetTimer;
    }

    public void resetGyro()
    {
        this.gyro.reset();
    }

    public boolean canDrive()
    {
        return resetTimer <= 0 && !this.gyro.isCalibrating();
    }

    public AHRS getGyro()
    {
        return this.gyro;
    }

    public List<SwerveModule> getModules()
    {
        return this.modules;
    }
}
