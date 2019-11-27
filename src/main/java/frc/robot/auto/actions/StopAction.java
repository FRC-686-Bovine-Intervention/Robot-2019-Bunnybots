package frc.robot.auto.actions;

import frc.robot.Shooter;
import frc.robot.lib.util.DataLogger;

public class StopAction implements Action {

    Shooter shooter = Shooter.getInstance();
    boolean finished = false;

    public StopAction() {}

    @Override
    public void start() {
        finished = false;
    }

    @Override
    public void update() {
        System.out.println("Starting to shoot.");
        shooter.stop();
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void done() {
        System.out.println("Finished");
    }

    // @Override
    // public DataLogger getLogger() {
    //     return null;
    // }
    
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
	    }
    };
	
	@Override
	public DataLogger getLogger() { return logger; }

}