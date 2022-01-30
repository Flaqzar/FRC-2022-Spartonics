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

  /**The PID id used to determine what PID settings to use */
  private static final int PID_ID = 0;
  /**The delay in milliseconds before a report gets sent to DriverStation if an action fails */
  private static final int MS_DELAY = 30;
  /**Settings are {kF, kP, kI, kD}*/         // F    P     I    D
  private static final double[] PID_SETTINGS = {0d, 0.25d, 0d, 1d};

  /**Main falcon motor*/
  private final WPI_TalonFX falcon0 = new WPI_TalonFX(1);
  /**Main controller*/
  private final XboxController controller = new XboxController(0);
  private double falcon0Rotation = 0d;
  private double currentRotation = 0d;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit()
  {
    //? What does this do?
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption(kCustomAuto, kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //Motor settings stuff
    falcon0.configFactoryDefault();
    falcon0.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, PID_ID, MS_DELAY);
    falcon0.setSensorPhase(true);
    falcon0.setInverted(false);

    falcon0.configNominalOutputForward(0d, MS_DELAY);
    falcon0.configNominalOutputReverse(0d, MS_DELAY);
    falcon0.configPeakOutputForward(1d, MS_DELAY);
    falcon0.configPeakOutputReverse(-1d, MS_DELAY);

    falcon0.configAllowableClosedloopError(0, PID_ID, MS_DELAY);

    falcon0.config_kF(PID_ID, PID_SETTINGS[0], MS_DELAY);
    falcon0.config_kP(PID_ID, PID_SETTINGS[1], MS_DELAY);
    falcon0.config_kI(PID_ID, PID_SETTINGS[2], MS_DELAY);
    falcon0.config_kD(PID_ID, PID_SETTINGS[3], MS_DELAY);
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
    double leftXAxis = controller.getRawAxis(0);
    double leftYAxis = controller.getRawAxis(1);
    double leftJoystickDistance = Math.sqrt(leftXAxis * leftXAxis + leftYAxis * leftYAxis);

    //Creates a deadzone of 10%
    if(leftJoystickDistance < 0.1d)
    {
      leftXAxis = 0d;
      leftYAxis = 0d;
    }

    //currentRotation += currentRotation - (Math.atan(leftYAxis / leftXAxis) + (leftXAxis < 0d ? Math.PI : 0d) + (Math.PI / 2d));
    currentRotation = (Math.atan(leftYAxis / leftXAxis) + (leftXAxis < 0d ? Math.PI : 0d) + (Math.PI / 2d)) / (Math.PI * 2d);
    falcon0Rotation = currentRotation * 2048d;
    falcon0.set(ControlMode.Position, falcon0Rotation);
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

