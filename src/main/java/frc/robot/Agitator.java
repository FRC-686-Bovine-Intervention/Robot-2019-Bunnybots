package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.joystick.DriverControlsEnum;
import frc.robot.lib.joystick.SelectedDriverControls;



public class Agitator  
{
    public TalonSRX agitatorMotor;
	// singleton class
	private static Agitator instance = null;
	public static Agitator getInstance() 
	{ 
		if (instance == null) {
			instance = new Agitator();
		}
		return instance;
    }
    
    public final int kSlotIdx = 0;

    public final double kGearRatio = 21;
    public final double kCalMaxEncoderPulsePer100ms = 40650/kGearRatio;	// velocity at a max throttle (measured using Phoenix Tuner)
    public final double kCalMaxPercentOutput 		= 1.0;	// percent output of motor at above throttle (using Phoenix Tuner)

    public final double kCruiseVelocity = 0.80 * kCalMaxEncoderPulsePer100ms;		// cruise below top speed
	public final double kKf = kCalMaxPercentOutput * 1023.0 / kCalMaxEncoderPulsePer100ms;
	public double kKp = 0.002;	   
    public double kKi = 0.0;    
    public double kKd = 0;	// to resolve any overshoot, start at 10*Kp 


	public static double kQuadEncoderCodesPerRev = 1024;
	public static double kQuadEncoderUnitsPerRev = 4*kQuadEncoderCodesPerRev;
	public static double kQuadEncoderStatusFramePeriod = 0.100; // 100 ms

    public final int    kAllowableError = (int)rpmToEncoderUnitsPerFrame(10);

    public final int kPeakCurrentLimit = 50;
    public final int kPeakCurrentDuration = 200;
    public final int kContinuousCurrentLimit = 30;

    public final double kBallsPerSecond = 1;
    
    public Agitator() 
    {
        agitatorMotor = new TalonSRX(Constants.kAgitatorTalonId);

        //====================================================
        // Configure Deploy Motors
        //====================================================

        // Factory default hardware to prevent unexpected behavior
        agitatorMotor.configFactoryDefault();

		// configure encoder
		agitatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
		agitatorMotor.setSensorPhase(true); // set so that positive motor input results in positive change in sensor value
		agitatorMotor.setInverted(true);   // set to have green LEDs when driving forward
		
		// set relevant frame periods to be at least as fast as periodic rate
		agitatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,      (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		agitatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,    (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		agitatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		agitatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,  (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		
		// configure velocity loop PID 
        agitatorMotor.selectProfileSlot(kSlotIdx, Constants.kTalonPidIdx); 
        agitatorMotor.config_kF(kSlotIdx, kKf, Constants.kTalonTimeoutMs); 
        agitatorMotor.config_kP(kSlotIdx, kKp, Constants.kTalonTimeoutMs); 
        agitatorMotor.config_kI(kSlotIdx, kKi, Constants.kTalonTimeoutMs); 
        agitatorMotor.config_kD(kSlotIdx, kKd, Constants.kTalonTimeoutMs);
        agitatorMotor.configAllowableClosedloopError(kSlotIdx, kAllowableError, Constants.kTalonTimeoutMs);
        
        // current limits
        agitatorMotor.configPeakCurrentLimit(kPeakCurrentLimit, Constants.kTalonTimeoutMs);
        agitatorMotor.configPeakCurrentDuration(kPeakCurrentDuration, Constants.kTalonTimeoutMs);
        agitatorMotor.configContinuousCurrentLimit(kContinuousCurrentLimit, Constants.kTalonTimeoutMs);
        agitatorMotor.enableCurrentLimit(true);
    }

    public void setTarget(double rpm)
    {
        double encoderSpeed = rpmToEncoderUnitsPerFrame(rpm);
        agitatorMotor.set(ControlMode.Velocity, encoderSpeed);
        SmartDashboard.putNumber("Agitator Raw Speed", encoderSpeed);
    }

    public void setPIDValues(double P, double I, double D)
    {
        kKp = P;
        kKi = I;
        kKd = D;

        agitatorMotor.selectProfileSlot(kSlotIdx, Constants.kTalonPidIdx); 
        agitatorMotor.config_kF(kSlotIdx, kKf, Constants.kTalonTimeoutMs); 
        agitatorMotor.config_kP(kSlotIdx, kKp, Constants.kTalonTimeoutMs); 
        agitatorMotor.config_kI(kSlotIdx, kKi, Constants.kTalonTimeoutMs); 
        agitatorMotor.config_kD(kSlotIdx, kKd, Constants.kTalonTimeoutMs);

    }

    public void run()
    {
        SelectedDriverControls driverControls = SelectedDriverControls.getInstance();
        if (driverControls.getBoolean(DriverControlsEnum.SHOOT))
        {
            setTarget(((17.5*Math.PI)/(kBallsPerSecond*5+4.5)*60)/5);
        }
        else
        {
            setTarget(0);
        }
        SmartDashboard.putNumber("Current P", kKp);
        SmartDashboard.putNumber("Current I", kKi);
        SmartDashboard.putNumber("Current D", kKd);
    }

   	// Talon SRX reports position in rotations while in closed-loop Position mode
    public static double encoderUnitsToRevolutions(int _encoderPosition) {	return (double)_encoderPosition / (double)kQuadEncoderUnitsPerRev; }
    public static int revolutionsToEncoderUnits(double _rev) { return (int)(_rev * kQuadEncoderUnitsPerRev); }

    // Talon SRX reports speed in RPM while in closed-loop Speed mode
    public static double encoderUnitsPerFrameToRPM(int _encoderEdgesPerFrame) { return encoderUnitsToRevolutions(_encoderEdgesPerFrame) * 60.0 / kQuadEncoderStatusFramePeriod; }
    public static int rpmToEncoderUnitsPerFrame(double _rpm) { return (int)(revolutionsToEncoderUnits(_rpm) / 60.0 * kQuadEncoderStatusFramePeriod); }
   
   
}