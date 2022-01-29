package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Joystick;
// the axis is used for controlling triggers and joysticks
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot
{
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private double value = (90 / 360.0) * 2048;


  // defining mechanical aspects such as motors and pneumatics
  
  //sparks are under this label
  //private final Spark intakeMotor = new Spark(0);
  
  //falcons are under this one
  private final WPI_TalonFX turningMotor0= new WPI_TalonFX(0);

  // defining mechanical aspects such as motors and pneumatics
  //Spark testSpark = new Spark(0);


  private final XboxController m_joystick = new XboxController(0);
    
 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit()
  { //* Sorry for mess, will clean later

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption(kCustomAuto, kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    Gains kGains = new Gains(0.15, 0.0, 1.0, 0.0, 0, 1.0);

    /* Factory Default all hardware to prevent unexpected behavior */
		turningMotor0.configFactoryDefault();
		
		/* Config the sensor used for Primary PID and sensor direction */
        turningMotor0.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 
                                            0,
				                            30);

		/* Ensure sensor is positive when output is positive */
		turningMotor0.setSensorPhase(true);

		/**
		 * Set based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. 
		 */ 
		turningMotor0.setInverted(false);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // turningMotor0.setSensorPhase(true);

		/* Config the peak and nominal outputs, 12V means full */
		turningMotor0.configNominalOutputForward(0, 30);
		turningMotor0.configNominalOutputReverse(0, 30);
		turningMotor0.configPeakOutputForward(1, 30);
		turningMotor0.configPeakOutputReverse(-1, 30);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		turningMotor0.configAllowableClosedloopError(0, 0, 30);

		/* Config Position Closed Loop gains in slot0, typically kF stays zero. */
		turningMotor0.config_kF(0, kGains.kF, 30);
		turningMotor0.config_kP(0, kGains.kP, 30);
		turningMotor0.config_kI(0, kGains.kI, 30);
		turningMotor0.config_kD(0, kGains.kD, 30);
    }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    
    // defining controls

    double leftY = m_joystick.getLeftY();
    double leftX = m_joystick.getLeftX();
    double leftTrigger = m_joystick.getLeftTriggerAxis();
    boolean yButton = m_joystick.getYButtonPressed();
    boolean bButton = m_joystick.getBButtonPressed();
    //uncomment below to turn on intake mechanism
    //intakeMotor.set(1 * 0.85);

    value = value + util.XYposToRad(leftX, -leftY);
    turningMotor0.set(TalonFXControlMode.Position, value);
    System.out.println(value);

    //System.out.println(leftX + ", " + -leftY + ", " + util.XYposToRad(leftX,-leftY));
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit()
  {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic()
  {
    switch (m_autoSelected)
    {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

