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
import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.SelectedDriverControls;
import frc.robot.lib.util.DataLogController;
import frc.robot.lib.util.DataLogger;
import frc.robot.loops.LoopController;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

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
  private LoopController loopController;

	Drive drive = Drive.getInstance();
  Shooter shooter;
  Intake intake;
  Agitator agitator;
  SmartDashboardInteractions smartDashboardInteractions = SmartDashboardInteractions.getInstance();

  DataLogController robotLogger;
  OperationalMode operationalMode = OperationalMode.getInstance();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    shooter = Shooter.getInstance();
    intake = Intake.getInstance();
    agitator = Agitator.getInstance();


    loopController = new LoopController();
    loopController.register(Shooter.getInstance());
    loopController.register(Intake.getInstance());
    //loopController.register(Agitator.getInstance()); //Agitator is not yet set up with the loop interface


    SmartDashboard.putNumber("Pid",5);
    SmartDashboard.putNumber("pId",0);
    SmartDashboard.putNumber("piD",10000);
    selectedDriverControls.setDriverControls( smartDashboardInteractions.getDriverControlsSelection() );
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    shooter = Shooter.getInstance();
    SmartDashboard.putNumber("ShooterRPM", 0);
    SmartDashboard.putBoolean("Shooter Debug", false);
    SmartDashboard.putNumber("AgitatorDegree", 0);
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
  }
  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    selectedDriverControls.setDriverControls(smartDashboardInteractions.getDriverControlsSelection());
    agitator.run();
    shooter.run();
    loopController.run();	

    DriveCommand driveCmd = selectedDriverControls.getDriveCommand();
    drive.setOpenLoop(driveCmd);


    if (SmartDashboard.getBoolean("Agitator Debug", false))
    {
      double agitatorSet = SmartDashboard.getNumber("AgitatorDegree", 0);
      agitator.setDegree(agitatorSet);
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
