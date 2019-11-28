/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoModeExecuter;
import frc.robot.lib.joystick.SelectedDriverControls;
import frc.robot.lib.util.DataLogController;
import frc.robot.lib.util.DataLogger;
import frc.robot.subsystems.Intake;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  SelectedDriverControls selectedDriverControls = SelectedDriverControls.getInstance();
  private AutoModeExecuter autoModeExecuter = null;

  private Shooter shooter;
  private Intake intake = Intake.getInstance();
  private Agitator agitator = Agitator.getInstance();
  SmartDashboardInteractions smartDashboardInteractions = SmartDashboardInteractions.getInstance();

  DataLogController robotLogger;
  OperationalMode operationalMode = OperationalMode.getInstance();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    SmartDashboard.putNumber("Pid",0);
    SmartDashboard.putNumber("pId",0);
    SmartDashboard.putNumber("piD",0);
    selectedDriverControls.setDriverControls( smartDashboardInteractions.getDriverControlsSelection() );
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    shooter = Shooter.getInstance();
    SmartDashboard.putNumber("ShooterRPM", 0);
    SmartDashboard.putBoolean("Shooter Debug", false);
    SmartDashboard.putNumber("AgitatorRPM", 0);
    SmartDashboard.putBoolean("Agitator Debug", false);
    robotLogger = DataLogController.getRobotLogController();
    robotLogger.register(this.getLogger());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {

    operationalMode.set(OperationalMode.OperationalModeEnum.AUTONOMOUS);
    boolean logToFile = false;
    boolean logToSmartDashboard = true;
    robotLogger.setOutputMode(logToFile, logToSmartDashboard);

		Shuffleboard.startRecording();

    m_autoSelected = m_chooser.getSelected();
    selectedDriverControls.setDriverControls( smartDashboardInteractions.getDriverControlsSelection() );

    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    if (autoModeExecuter != null)
			{
    			autoModeExecuter.stop();
    		}
    		autoModeExecuter = null;
    		
			autoModeExecuter = new AutoModeExecuter();
      autoModeExecuter.setAutoMode(smartDashboardInteractions.getAutoModeSelection());
      autoModeExecuter.start();
  }
  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }
  @Override
  public void teleopInit() {
    double Pid = SmartDashboard.getNumber("Pid", 100);
    double pId = SmartDashboard.getNumber("pId", 0);
    double piD = SmartDashboard.getNumber("piD", 0);
    agitator.setPIDValues(Pid, pId, piD);
  }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    selectedDriverControls.setDriverControls(smartDashboardInteractions.getDriverControlsSelection());
    agitator.run();
    if (SmartDashboard.getBoolean("Shooter Debug", false))
    {
      double rpm = SmartDashboard.getNumber("ShooterRPM", 0);
      shooter.setTarget(rpm);
    }
    shooter.run();
    if (SmartDashboard.getBoolean("Agitator Debug", false))
    {
      double agitatorSet = SmartDashboard.getNumber("AgitatorRPM", 0);
      agitator.setTarget(agitatorSet);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
			put("OperationalMode", operationalMode.get().toString());
        }
    };
    
    public DataLogger getLogger() { return logger; }

}
