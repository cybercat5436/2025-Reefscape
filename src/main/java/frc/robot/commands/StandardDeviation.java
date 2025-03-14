// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.PoseUpdater;

//At 1.2m, the Pose is (6.52, 4.026) Standard Deviation: 0.25
//At 2.6m, the Pose is (7.82, 4.026) Standard Deviation: 0.537

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StandardDeviation extends Command {
  private PoseUpdater poseUpdater;
  private CommandSwerveDrivetrain commandSwerveDrivetrain;
  private ArrayList<Double> distancesX = new ArrayList();
  private ArrayList<Double> distancesY = new ArrayList();
  private double distanceEstimateX;
  private double distanceEstimateY;
  private Pose2d robotPose;
  private LimeLight limeLightFront;
  private LimeLight limeLightRight;
  /** Creates a new StandardDeviation. */

  public StandardDeviation(PoseUpdater poseUpdater, CommandSwerveDrivetrain commandSwerveDrivetrain, Pose2d robotPose, LimeLight limeLightFront, LimeLight limeLightRight) {
    this.poseUpdater = poseUpdater;
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.robotPose = robotPose;
    this.limeLightFront = limeLightFront;
    this.limeLightRight = limeLightRight;
    
  }
   
    // Use addRequirements() here to declare subsystem dependencies

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    commandSwerveDrivetrain.resetPose(robotPose);
    LimelightHelpers.setPipelineIndex(limeLightFront.limelightName, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d visionPose = poseUpdater.visionDataCollection(limeLightFront);
    distanceEstimateX = commandSwerveDrivetrain.getState().Pose.getX() - visionPose.getX();
    distancesX.add(distanceEstimateX);
    distanceEstimateY = commandSwerveDrivetrain.getState().Pose.getY() - visionPose.getY();
    distancesY.add(distanceEstimateY);

    System.out.println("distanceEstimateX: " + distanceEstimateX);
    System.out.println("distanceEstimateY: " + distanceEstimateY);
    System.out.println(robotPose);
    System.out.println("Vision Pose: " + visionPose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // Standard Deviation of X Values

    double sum = 0;
    for (double x:distancesX) {
      sum += x;
    }
    double meanCalculationX = sum/distancesX.size();

    double sum2 = 0;
    for (double x:distancesX) {
      sum2 += Math.pow((meanCalculationX - x),2);

    }
    
    double result = Math.sqrt(sum2/(distancesX.size() - 1));
    

    // Standard Deviation of Y Values
    
    double sum3 = 0;
    for (double y:distancesY) {
      sum3 += y;
    }
    double meanCalculationY = sum3/distancesY.size();

    double sum4 = 0;
    for (double y:distancesY) {
      sum4 += Math.pow((meanCalculationY - y),2);

    }
    double result2 = Math.sqrt(sum4/(distancesY.size() - 1));

    System.out.println("**********XStandardDeviation: " + result);
    System.out.println("**********XAverage: " + meanCalculationX);
    System.out.println("**********YStandardDeviation: " + result2);
    System.out.println("**********YAverage: " + meanCalculationY);
    
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
