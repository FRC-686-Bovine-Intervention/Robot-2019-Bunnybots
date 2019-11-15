package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.PIDController;
import frc.robot.loops.DriveLoop;



public class Agitator extends Subsystem 
{
    public TalonSRX agitatorMotor = new TalonSRX(/*deviceNumber*/0);
	// singleton class
	private static Agitator instance = null;
	public static Agitator getInstance() 
	{ 
		if (instance == null) {
			instance = new Agitator();
		}
		return instance;
	}

    public void setAgitator(double kAgitatorSpeed)
    {
        agitatorMotor.set(ControlMode.PercentOutput, kAgitatorSpeed);
    }

    @Override
    public void stop() {
        agitatorMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zeroSensors() {

    }
}