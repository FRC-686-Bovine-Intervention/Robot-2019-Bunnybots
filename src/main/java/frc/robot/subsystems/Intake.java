//initialize intake; turn on the motors-forward or reverse;

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants;


public class Intake extends Subsystem 
{
    public TalonSRX intakeMotor = new TalonSRX(Constants.kIntakeTalonId);
	// singleton class
	private static Intake instance = null;
	public static Intake getInstance() 
	{ 
		if (instance == null) {
			instance = new Intake();
		}
		return instance;
	}

    public void setIntake()
    {
        intakeMotor.set(ControlMode.PercentOutput, Constants.kIntakeSpeed);
    }

    @Override
    public void stop() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zeroSensors() {

    }
}