package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.SetShooterSpeedAction;
import frc.robot.auto.actions.ShootBallAction;
import frc.robot.auto.actions.StopShooterAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.subsystems.Shooter;

public class BunnybotsAuto extends AutoModeBase {

    public BunnybotsAuto() 
    { 
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        // Shoot Bunny at high goal
        runAction(new SetShooterSpeedAction(Shooter.GoalEnum.BUNNY_HIGH_GOAL));
        runAction(new WaitAction(0.5));     // wait for  motor to change speeds
        int numBalls = 1;
        runAction(new ShootBallAction(numBalls));
        runAction(new WaitAction(2.0));     // wait for  motor to change speeds

        // Shoot at high goal
        runAction(new SetShooterSpeedAction(Shooter.GoalEnum.HIGH_GOAL));
        runAction(new WaitAction(0.5));     // wait for  motor to change speeds
        numBalls = 2;
        runAction(new ShootBallAction(numBalls));
        runAction(new WaitAction(2.0));     // wait for  motor to change speeds

        // this is where we move to the low goal

        // Shoot at low goal
        runAction(new SetShooterSpeedAction(Shooter.GoalEnum.LOW_GOAL));
        runAction(new WaitAction(0.5));     // wait for  motor to change speeds
        numBalls = 2;
        runAction(new ShootBallAction(numBalls));
        runAction(new WaitAction(2.0));     // wait for  motor to change speeds

        runAction(new StopShooterAction());


    }
}