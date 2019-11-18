package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.sensors.Limelight;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.Vector2d;

public class Shooter
{
	// singleton class
    private static Shooter instance = null;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    //====================================================
    // Members
    //====================================================
    public TalonSRX shooterMotor;
    public Limelight camera;

    //====================================================
    // Constants
    //====================================================
    
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

    public final int kPeakCurrentLimit = 50;
    public final int kPeakCurrentDuration = 200;
    public final int kContinuousCurrentLimit = 30;

    public double kHighGoalHeight = 76.75;
    public double kLowGoalHeight = 41.5;
    public double kCameraHeight = 16;
    public double kCameraAngle = 35;

    // Distance vs. RPM Tables
    public double highGoalTable[][] = {
        {36,    2000},
        {48,    3000},
        {60,    4000} };


    public Shooter() 
    {
        shooterMotor = new TalonSRX(3);
        camera = new Limelight("limelight", 0);

        //====================================================
        // Configure Deploy Motors
        //====================================================

        // Factory default hardware to prevent unexpected behavior
        shooterMotor.configFactoryDefault();

		// configure encoder
		shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
		shooterMotor.setSensorPhase(true); // set so that positive motor input results in positive change in sensor value
		shooterMotor.setInverted(true);   // set to have green LEDs when driving forward
		
		// set relevant frame periods to be at least as fast as periodic rate
		shooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,      (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		shooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,    (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		shooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		shooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,  (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		
		// configure velocity loop PID 
        shooterMotor.selectProfileSlot(kSlotIdx, Constants.kTalonPidIdx); 
        shooterMotor.config_kF(kSlotIdx, kKf, Constants.kTalonTimeoutMs); 
        shooterMotor.config_kP(kSlotIdx, kKp, Constants.kTalonTimeoutMs); 
        shooterMotor.config_kI(kSlotIdx, kKi, Constants.kTalonTimeoutMs); 
        shooterMotor.config_kD(kSlotIdx, kKd, Constants.kTalonTimeoutMs);
        shooterMotor.configAllowableClosedloopError(kSlotIdx, kAllowableError, Constants.kTalonTimeoutMs);
        
        // current limits
        shooterMotor.configPeakCurrentLimit(kPeakCurrentLimit, Constants.kTalonTimeoutMs);
        shooterMotor.configPeakCurrentDuration(kPeakCurrentDuration, Constants.kTalonTimeoutMs);
        shooterMotor.configContinuousCurrentLimit(kContinuousCurrentLimit, Constants.kTalonTimeoutMs);
        shooterMotor.enableCurrentLimit(true);
    }
    


    public void setTarget(double rpm)
    {
        double encoderSpeed = rpmToEncoderUnitsPerFrame(rpm);
        shooterMotor.set(ControlMode.Velocity, encoderSpeed);
        SmartDashboard.putNumber("Shooter Raw Speed", encoderSpeed);
    }
    
	// Talon SRX reports position in rotations while in closed-loop Position mode
	public static double encoderUnitsToRevolutions(int _encoderPosition) {	return (double)_encoderPosition / (double)kQuadEncoderUnitsPerRev; }
	public static int revolutionsToEncoderUnits(double _rev) { return (int)(_rev * kQuadEncoderUnitsPerRev); }

	// Talon SRX reports speed in RPM while in closed-loop Speed mode
	public static double encoderUnitsPerFrameToRPM(int _encoderEdgesPerFrame) { return encoderUnitsToRevolutions(_encoderEdgesPerFrame) * 60.0 / kQuadEncoderStatusFramePeriod; }
	public static int rpmToEncoderUnitsPerFrame(double _rpm) { return (int)(revolutionsToEncoderUnits(_rpm) / 60.0 * kQuadEncoderStatusFramePeriod); }



    public void run()
    {
        double targetDeg = camera.getTargetVerticalAngleRad() * Vector2d.radiansToDegrees;
        double distance = handleDistance(camera.getTargetVerticalAngleRad(), false);
        SmartDashboard.putNumber("Target Degree", targetDeg);
        SmartDashboard.putNumber("Distance To Target", distance);
        SmartDashboard.putBoolean("Found Target", camera.getIsTargetFound());
    }

    public double handleDistance (double angleRad, boolean lowGoal)
    {
        if (lowGoal)
        {
            return (kLowGoalHeight-kCameraHeight)/(Math.tan(angleRad+(kCameraAngle * Vector2d.degreesToRadians)));
        }
        else
        {
            return (kHighGoalHeight-kCameraHeight)/(Math.tan(angleRad+(kCameraAngle * Vector2d.degreesToRadians)));
        }
    }
    
	private final DataLogger logger = new DataLogger()
	{
		@Override
		public void log()
		{
            // put("Shooter/Deploy/targetPosition", targetPosition.toString());

		}
	}; 
    
	public DataLogger getLogger()
	{
		return logger;
	}    
}