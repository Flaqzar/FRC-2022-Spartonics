package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// the axis is used for controlling triggers and joysticks
import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * represents a swerve module. Wraps two falcon500s, one for driving and one for
 * steering
 */
public class swerveModule {
    // Motor objects used in each swerve drive module
    WPI_TalonFX driveFalcon;
    WPI_TalonFX steeringFalcon;
    double lastAngle;
    // TODO: add the CANcoder

    /** The PID id used to determine what PID settings to use */
    private static final int PID_ID = 0;
    /**
     * The delay in milliseconds before a report gets sent to DriverStation if an
     * action fails
     */
    private static final int MS_DELAY = 30;
    /** Settings are {kF, kP, kI, kD} */ // F P I D
    private static final double[] PID_SETTINGS = { 0d, 0.25d, 0d, 1d };

    public swerveModule(int driveMotor, int steeringMotor) {
        // get the motor objects from the CAN bus
        driveFalcon = new WPI_TalonFX(driveMotor);
        steeringFalcon = new WPI_TalonFX(steeringMotor);

    }

    /**
     * initialize the motors. Call this before doing stuff with the motor.
     */
    public void init() {
        // Motor settings stuff
        steeringFalcon.configFactoryDefault();
        steeringFalcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, PID_ID, MS_DELAY);
        steeringFalcon.setSensorPhase(true);
        steeringFalcon.setInverted(false);

        steeringFalcon.configNominalOutputForward(0d, MS_DELAY);
        steeringFalcon.configNominalOutputReverse(0d, MS_DELAY);
        steeringFalcon.configPeakOutputForward(1d, MS_DELAY);
        steeringFalcon.configPeakOutputReverse(-1d, MS_DELAY);

        steeringFalcon.configAllowableClosedloopError(0, PID_ID, MS_DELAY);

        steeringFalcon.config_kF(PID_ID, PID_SETTINGS[0], MS_DELAY);
        steeringFalcon.config_kP(PID_ID, PID_SETTINGS[1], MS_DELAY);
        steeringFalcon.config_kI(PID_ID, PID_SETTINGS[2], MS_DELAY);
        steeringFalcon.config_kD(PID_ID, PID_SETTINGS[3], MS_DELAY);

    }

    /**
     * set the angle of the motor
     * 
     * @param angle desired angle of the motor
     */
    public void setAngle(int angle) {
        // if the last angle exists, don't rotate more than 180 degrees
        if (angle != null) {
            if (math.abs(lastAngle - angle) < 180) {
                steeringFalcon.set(ControlMode.Position, angle / 360 * 2048);
            } else {
                steeringFalcon.set(ControlMode.Position, (360 - angle) / 360 * 2048);
            }
            // if it doesn't, just spin to the angle
        }else {
            steeringFalcon.set(ControlMode.Position, angle / 360 * 2048);
        }
        lastAngle = angle;
    }

}
