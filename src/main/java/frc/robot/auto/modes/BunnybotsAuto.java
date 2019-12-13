package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.CurvedDriveAction;
import frc.robot.auto.actions.LimelightLEDAction;
import frc.robot.auto.actions.SetShooterSpeedAction;
import frc.robot.auto.actions.ShootBallAction;
import frc.robot.auto.actions.StopShooterAction;
import frc.robot.auto.actions.VisionDriveAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.lib.sensors.Limelight.LedMode;
import frc.robot.subsystems.Shooter;

public class BunnybotsAuto extends AutoModeBase {

    public BunnybotsAuto() 
    { 
    }
    
    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Start BunnybotsAuto");
        // Turn on Limelight
        runAction(new LimelightLEDAction(LedMode.kOn));

        // Shoot Bunny at high goal
        System.out.println("Set Shooter Speed for High Goal, Bunny Ball");
        runAction(new SetShooterSpeedAction(Shooter.GoalEnum.BUNNY_HIGH_GOAL));
        System.out.println("Shoot 1 Ball");
        int numBalls = 1;
        runAction(new ShootBallAction(numBalls));
        System.out.println("Wait for Ball to Shoot");
        runAction(new WaitAction(2.0));     // wait for  motor to change speeds

        // Shoot at high goal
        System.out.println("Set Shooter Speed for High Goal");
        runAction(new SetShooterSpeedAction(Shooter.GoalEnum.HIGH_GOAL));
        System.out.println("Shoot 2 Balls");
        numBalls = 2;
        runAction(new ShootBallAction(numBalls));
        System.out.println("Wait for Ball to Shoot");
        runAction(new WaitAction(2.0));     // wait for  motor to change speeds

        // this is where we move to the low goal
        runAction(new CurvedDriveAction(-0.3, 31.25, -90));
        runAction(new CurvedDriveAction(-0.3, -31.25, 30));
        runAction(new VisionDriveAction(0.25));

        // Shoot at low goal
        System.out.println("Set Shooter Speed for Low Goal");
        runAction(new SetShooterSpeedAction(Shooter.GoalEnum.LOW_GOAL));
        System.out.println("Shoot 2 Balls");
        numBalls = 2;
        runAction(new ShootBallAction(numBalls));
        System.out.println("Wait for Ball to Shoot");
        runAction(new WaitAction(2.0));     // wait for  motor to change speeds

        System.out.println("Stop Shooter");
        runAction(new StopShooterAction());
        
        // Turn off Limelight
        runAction(new LimelightLEDAction(LedMode.kOff));
    }
}