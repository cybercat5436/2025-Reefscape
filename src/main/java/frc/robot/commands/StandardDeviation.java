// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PoseUpdater;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StandardDeviation extends Command {
  private PoseUpdater poseUpdater;
  private CommandSwerveDrivetrain commandSwerveDrivetrain;
  private ArrayList<Double> distances = new ArrayList();
  private double distanceEstimate;
  /** Creates a new StandardDeviation. */

  public StandardDeviation(PoseUpdater poseUpdater, CommandSwerveDrivetrain commandSwerveDrivetrain) {
    this.poseUpdater = poseUpdater;
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
  }
   
    // Use addRequirements() here to declare subsystem dependencies

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distanceEstimate = commandSwerveDrivetrain.getState().Pose.getTranslation().getDistance(poseUpdater.visionDataCollection().getTranslation());
    distances.add(distanceEstimate);
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double sum = 0;
    for (double x:distances) {
      sum += x;
    }
    double meanCalculation = sum/distances.size();

    double sum2 = 0;
    for (double x:distances) {
      sum2 += Math.pow((meanCalculation - x),2);

    }
    
    double result = Math.sqrt(sum2/(distances.size() - 1));
    System.out.println("**********Result: " + result);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
