package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.SteeringLib.DeadbandNonLinearity;
import frc.robot.lib.joystick.SteeringLib.NonLinearityEnum;
import frc.robot.lib.util.DataLogger;



public class DriverControlsXbox extends DriverControlsBase
{
    // Joystick Port Constants
    public static int kXboxStickPort   = 0;
    public static int kButtonBoardPort = 1;

    public static JoystickBase stick;
    public static JoystickBase buttonBoard;
    public static JoystickBase lStick;

    public static SteeringBase steeringControls;

    // button board constants
    public static int kCargoIntakeRocketButton =    ButtonBoard.kButtonB;
    public static int kCargoIntakeCargoShipButton = ButtonBoard.kButtonA;
    public static int kDefenseButton =              ButtonBoard.kButtonRB;
    public static int kClimbingStartButton =        ButtonBoard.kButtonLB;
    public static int kClimbingExtendButton =       ButtonBoard.kButtonX;
    public static int kClimbingRetractButton =      ButtonBoard.kButtonY;
    public static int kEmergencyZeroingAxis =       ButtonBoard.kButtonSR;

    public DriverControlsXbox() 
    {
        stick = new Thrustmaster(kXboxStickPort);
        buttonBoard = new ButtonBoard(kButtonBoardPort);

        double throttleDeadband =     0.02;
        double turnDeadband =         0.02;
        NonLinearityEnum throttleNonLinearity = NonLinearityEnum.SQUARED;
        NonLinearityEnum turnNonLinearity =     NonLinearityEnum.SQUARED;

        steeringControls = new XboxArcadeDriveSteering(stick, new DeadbandNonLinearity(throttleDeadband, turnDeadband, throttleNonLinearity, turnNonLinearity));
    }

    public DriveCommand getDriveCommand() 
    {
        return steeringControls.getDriveCommand(); 
    }


    public boolean getBoolean( DriverControlsEnum _control ) 
    {
        switch (_control)
        {
            case INTAKE:                        return false;
            case OUTTAKE:                       return false;
            case HIGH_SHOOT:                    return lStick.getButton(Thrustmaster.kTriggerButton);
            case LOW_SHOOT:                     return false;
            case QUICK_TURN:                    return false;
            default:                            return false;
        }
    }







    public DataLogger getLogger() { return logger; }
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            if (stick != null)              { stick.getLogger().log(); }
            if (buttonBoard != null)        { buttonBoard.getLogger().log(); }
            if (steeringControls != null)   { steeringControls.getLogger().log(); }
         }
    };            
}
