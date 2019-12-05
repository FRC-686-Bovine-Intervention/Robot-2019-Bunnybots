package frc.robot;



/**
 * ElevatorArmBar status structure, filled by ElevatorLoop.java
 */
public class AgitatorState
{
	private static AgitatorState instance = new AgitatorState();
	public static AgitatorState getInstance() { return instance; }	
	
	// all member variables should be private to force other object to use the set/get access methods
	// which are synchronized to allow multi-thread synchronization

	private double positionInches = 0.0; //change to angle
	private double velocityInchesPerSec = 0.0;

	private double trajectoryTargetInches = 0.0;
	private double trajectoryPositionInches = 0.0;
	private double trajectoryVelocityInchesPerSec = 0.0;

	private double pidError = 0.0;
	private double motorPercentOutput = 0.0;
	private double motorCurrent = 0.0;

	private boolean limitSwitchTriggered = false;
	
	
	public AgitatorState() {}
	
	public synchronized double getPositionInches() {
		return positionInches;
	}

	public synchronized void setPositionInches(double positionInches) {
		this.positionInches = positionInches;
	}

	public synchronized double getVelocityInchesPerSec() {
		return velocityInchesPerSec;
	}

	public synchronized void setVelocityInchesPerSec(double velocityInchesPerSec) {
		this.velocityInchesPerSec = velocityInchesPerSec;
	}

	public synchronized double getTrajectoryTargetInches() {
		return trajectoryTargetInches;
	}

	public synchronized void setTrajectoryTargetInches(double trajectoryTargetInches) {
		this.trajectoryTargetInches = trajectoryTargetInches;
	}

	public synchronized double getTrajectoryPositionInches() {
		return trajectoryPositionInches;
	}

	public synchronized void setTrajectoryPositionInches(double trajectoryPositionInches) {
		this.trajectoryPositionInches = trajectoryPositionInches;
	}

	public synchronized double getTrajectoryVelocityInchesPerSec() {
		return trajectoryVelocityInchesPerSec;
	}

	public synchronized void setTrajectoryVelocityInchesPerSec(double trajectoryVelocityInchesPerSec) {
		this.trajectoryVelocityInchesPerSec = trajectoryVelocityInchesPerSec;
	}

	public synchronized double getPidError() {
		return pidError;
	}

	public synchronized void setPidError(double pidError) {
		this.pidError = pidError;
	}

	public synchronized double getMotorPercentOutput() {
		return motorPercentOutput;
	}

	public synchronized void setMotorPercentOutput(double motorPercentOutput) {
		this.motorPercentOutput = motorPercentOutput;
	}

	public synchronized double getMotorCurrent() {
		return motorCurrent;
	}

	public synchronized void setMotorCurrent(double motorCurrent) {
		this.motorCurrent = motorCurrent;
	}
	
	public synchronized boolean isLimitSwitchTriggered() {
		return limitSwitchTriggered;
	}

	public synchronized void setLimitSwitchTriggered(boolean limitSwitchTriggered) {
		this.limitSwitchTriggered = limitSwitchTriggered;
	}	
}
