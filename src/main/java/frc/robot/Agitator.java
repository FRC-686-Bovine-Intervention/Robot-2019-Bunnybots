package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



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

    public final double kCalMaxEncoderPulsePer100ms = 33300;	// velocity at a max throttle (measured using Phoenix Tuner)
    public final double kCalMaxPercentOutput 		= 1.0;	// percent output of motor at above throttle (using Phoenix Tuner)

    public final double kCruiseVelocity = 0.80 * kCalMaxEncoderPulsePer100ms;		// cruise below top speed
	public final double kKf = kCalMaxPercentOutput * 1023.0 / kCalMaxEncoderPulsePer100ms;
	public final double kKp = 0.025;	   
	public final double kKd = 9.0;	// to resolve any overshoot, start at 10*Kp 
	public final double kKi = 0.0;    

	public static double kQuadEncoderCodesPerRev = 1024;
	public static double kQuadEncoderUnitsPerRev = 4*kQuadEncoderCodesPerRev;
	public static double kQuadEncoderStatusFramePeriod = 0.100; // 100 ms

    public final int    kAllowableError = (int)rpmToEncoderUnitsPerFrame(10);

    	// Talon SRX reports position in rotations while in closed-loop Position mode
	public static double encoderUnitsToRevolutions(int _encoderPosition) {	return (double)_encoderPosition / (double)kQuadEncoderUnitsPerRev; }
	public static int revolutionsToEncoderUnits(double _rev) { return (int)(_rev * kQuadEncoderUnitsPerRev); }

	// Talon SRX reports speed in RPM while in closed-loop Speed mode
	public static double encoderUnitsPerFrameToRPM(int _encoderEdgesPerFrame) { return encoderUnitsToRevolutions(_encoderEdgesPerFrame) * 60.0 / kQuadEncoderStatusFramePeriod; }
	public static int rpmToEncoderUnitsPerFrame(double _rpm) { return (int)(revolutionsToEncoderUnits(_rpm) / 60.0 * kQuadEncoderStatusFramePeriod); }

    public final int kPeakCurrentLimit = 50;
    public final int kPeakCurrentDuration = 200;
    public final int kContinuousCurrentLimit = 30;


    
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
        SmartDashboard.putNumber("Shooter Raw Speed", encoderSpeed);
    }

}