//initialize intake; turn on the motors-forward or reverse;

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Constants;
import frc.robot.lib.joystick.DriverControlsBase;
import frc.robot.lib.joystick.DriverControlsEnum;
import frc.robot.lib.joystick.SelectedDriverControls;
import frc.robot.loops.Loop;


public class Intake extends Subsystem implements Loop
{
    public VictorSPX intakeMotor;

	// singleton class
	private static Intake instance = null;
	public static Intake getInstance() 
	{ 
		if (instance == null) {
			instance = new Intake();
		}
		return instance;
    }


    public Intake(){
        intakeMotor = new VictorSPX(Constants.kIntakeTalonId);

       // Factory default hardware to prevent unexpected behavior
       intakeMotor.configFactoryDefault();

    }
    

    //Loop functions
    @Override
    public void onStart() {
        stop();
    }

    @Override
    public void onLoop() {
        DriverControlsBase driverControls = SelectedDriverControls.getInstance().get();

        if (driverControls.getBoolean(DriverControlsEnum.INTAKE))
        {
            set(+Constants.kIntakeVoltage);
        }
        else if (driverControls.getBoolean(DriverControlsEnum.OUTTAKE))
        {
            set(-Constants.kIntakeVoltage);
        }
        else 
        {
            stop();
        }
    }

    @Override
    public void onStop() {
        stop();
    }



    public void set(double speed)
    {
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public void stop() {
        set(0.0);
    }

    @Override
    public void zeroSensors() {

    }
}