package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * represents a swerve module. Wraps two falcon500s, one for driving and one for
 * steering
 */
public class SwerveModule {
	// Motor objects used in each swerve drive module
	WPI_TalonFX driveFalcon;
	WPI_TalonFX steeringFalcon;
	double lastAngle;
	// TODO: add the CANcoder

	/** The PID id used to determine what PID settings to use */
	private static final int PID_ID = 0;
	/** Rotation measured in radians. */
	double currentRotation = 0d;

	public SwerveModule(int driveMotor, int steeringMotor) {
		// get the motor objects from the CAN bus
		this.driveFalcon = new WPI_TalonFX(driveMotor);
		this.steeringFalcon = new WPI_TalonFX(steeringMotor);
	}

	/**
	 * initialize the motors. Call this before doing stuff with the motor.
	 */
	public void init() {
		// Motor settings stuff
		this.steeringFalcon.configFactoryDefault();
		this.steeringFalcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, PID_ID,
				Constants.MS_DELAY);
		this.steeringFalcon.setSensorPhase(true);
		this.steeringFalcon.setInverted(false);

		// no idea what this does.
		this.steeringFalcon.configNominalOutputForward(0d, Constants.MS_DELAY);
		this.steeringFalcon.configNominalOutputReverse(0d, Constants.MS_DELAY);
		this.steeringFalcon.configPeakOutputForward(1d, Constants.MS_DELAY);
		this.steeringFalcon.configPeakOutputReverse(-1d, Constants.MS_DELAY);

		this.steeringFalcon.configAllowableClosedloopError(0, PID_ID, Constants.MS_DELAY);

		// configure PID tuning
		this.steeringFalcon.config_kF(PID_ID, Constants.PID_SETTINGS[0], Constants.MS_DELAY);
		this.steeringFalcon.config_kP(PID_ID, Constants.PID_SETTINGS[1], Constants.MS_DELAY);
		this.steeringFalcon.config_kI(PID_ID, Constants.PID_SETTINGS[2], Constants.MS_DELAY);
		this.steeringFalcon.config_kD(PID_ID, Constants.PID_SETTINGS[3], Constants.MS_DELAY);

		// reset angle
		this.steeringFalcon.set(ControlMode.Position, this.currentRotation);
	}

	public void setAngleFromJoystick(double x, double y)
	{
		double joystickAngle = (Math.atan(y / x) + (x < 0d ? Math.PI : 0d) + Constants.PI_OVER_TWO) / Constants.TWO_PI;
		
		lerpedAngle = ((((oldpos-newpos) % 2pi)+pi) % 2pi) -pi

		this.steeringFalcon.set(ControlMode.Position, this.currentRotation / Constants.TWO_PI * 2048d);
	}

	/**
	 * set the angle of the motor
	 * 
	 * @param angle desired angle of the motor in Radians
	 * 
	 */
	public void setAngle(double angle) {
		/*
		 * equations here are taken from:
		 * https://www.desmos.com/calculator/rafir51bn0
		*/
		double rotate = this.lastAngle-angle;
		rotate = (rotate % Constants.TWO_PI) + Math.PI;
		rotate = (rotate % Constants.TWO_PI) - Math.PI;
		
		this.steeringFalcon.set(ControlMode.Position, lastAngle + angle / Constants.TWO_PI * 2048);
		

		this.lastAngle = angle;
	}
}
