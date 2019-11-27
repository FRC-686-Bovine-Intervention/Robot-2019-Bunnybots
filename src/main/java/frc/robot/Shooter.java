package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.joystick.DriverAxisEnum;
import frc.robot.lib.joystick.DriverControlsEnum;
import frc.robot.lib.joystick.SelectedDriverControls;
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
    public double speed;

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
    public final int kSliderMax = 200;

    public double kHighGoalHeight = 76.75;
    public double kLowGoalHeight = 41.5;
    public double kCameraHeight = 16;
    public double kCameraAngle = 42.5;
    public double kFrontToCameraDist = 7;

    // Distance vs. RPM Tables
    public double highGoalTable[][] = {
        {30,    2900},
        {39,    2750},
        {47,    2700},
        {56,    2700},
        {59,    2750},
        {79,    2825},   };

    public double lowGoalTable[][] = {
        {43,    1950},
        {56,    2050}, 
        {72,    1800},
        {88,    1890},
        {105,   2000},
        {117,   2175}};


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
        SelectedDriverControls driverControls = SelectedDriverControls.getInstance();
        double targetDeg = camera.getTargetVerticalAngleRad() * Vector2d.radiansToDegrees;
        double distance = handleDistance(camera.getTargetVerticalAngleRad(), false)-kFrontToCameraDist;
        boolean targetLowGoal = driverControls.getBoolean(DriverControlsEnum.TARGET_LOW);
        double goalTable[][] = handleGoalTable(targetLowGoal, highGoalTable, lowGoalTable);
        int keyL = getLinear(distance, goalTable);
        double shooterCorrection = -driverControls.getAxis(DriverAxisEnum.SHOOTER_SPEED_CORRECTION)*kSliderMax;
        double nominalSpeed = handleLinear(distance, goalTable[keyL][0], goalTable[keyL+1][0], goalTable[keyL][1], goalTable[keyL+1][1]);
        if (camera.getIsTargetFound())
        {
            speed = nominalSpeed + shooterCorrection;
            SmartDashboard.putNumber("Distance To Target", distance);
        }
        if (!SmartDashboard.getBoolean("Shooter Debug", false))
        {
            if (driverControls.getBoolean(DriverControlsEnum.SHOOT))
            {
                setTarget(speed);
            }
            else
            {
                setTarget(0);
            }
        }
        SmartDashboard.putNumber("Shooter Speed", speed);
        SmartDashboard.putNumber("CorrectionValue", shooterCorrection);
        SmartDashboard.putNumber("NominalSpeed", nominalSpeed);
        SmartDashboard.putNumber("Target Degree", targetDeg);
        SmartDashboard.putBoolean("Found Target", camera.getIsTargetFound());
    }

    public void shoot(boolean lowGoal)
    {
        double goalTable[][] = handleGoalTable(lowGoal, highGoalTable, lowGoalTable);
        double distance = handleDistance(camera.getTargetVerticalAngleRad(), false)-kFrontToCameraDist;
        int keyL = getLinear(distance, goalTable);
        double nominalSpeed = handleLinear(distance, goalTable[keyL][0], goalTable[keyL+1][0], goalTable[keyL][1], goalTable[keyL+1][1]);
        setTarget(nominalSpeed);
    }

    public void stop() // You shall not PAAAAASSSSS -Gandalf the Code
    {
        setTarget(0);
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

    public double[][] handleGoalTable (boolean lowGoal, double[][] falseTable, double[][] trueTable)
    {
        if (lowGoal)
        {
            return trueTable;
        }
        else
        {
            return falseTable;
        }
    }

    public int getLinear (double d, double table[][])
    {
        double distance = Math.max(Math.min(d, table[table.length-1][0]), table[0][0]);
        int k;
        for (k=0;k<table.length;k++)
        {
            if (distance <= table[k][0])
            {
                break;
            }
        }
        return Math.max(k-1, 0);
    }

    public double handleLinear (double d, double dL, double dH, double sL, double sH)
    {
        return (sH-sL)*Math.min((d-dL)/(dH-dL),1)+sL;
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