//initialize intake; turn on the motors-forward or reverse;

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.PIDController;
import frc.robot.loops.DriveLoop;



public class Intake extends Subsystem 
{
    public TalonSRX intakeMotor = new TalonSRX(/*deviceNumber*/0);
	// singleton class
	private static Intake instance = null;
	public static Intake getInstance() 
	{ 
		if (instance == null) {
			instance = new Intake();
		}
		return instance;
	}

    public void setIntake(double kzoom)
    {
        intakeMotor.set(ControlMode.PercentOutput, kzoom);
    }

    @Override
    public void stop() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zeroSensors() {

    }
}