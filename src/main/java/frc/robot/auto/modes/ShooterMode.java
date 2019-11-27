package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.*;

public class ShooterMode extends AutoModeBase {

    public ShooterMode() 
    { 
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new ShootHighGoalAction());
        runAction(new WaitAction(5));
        runAction(new ShootLowGoalAction());
        runAction(new WaitAction(5));
    }
}