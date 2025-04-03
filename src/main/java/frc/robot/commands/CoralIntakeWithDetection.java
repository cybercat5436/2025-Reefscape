// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.GamePieceDetector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntakeWithDetection extends Command {
  /** Creates a new CoralIntakeWithDetection. */
  private Coral coral;
  private GamePieceDetector coralDetector;
  public CoralIntakeWithDetection(Coral coral, GamePieceDetector coralDetector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.coral = coral;
    this.coralDetector= coralDetector;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coral.shoot(-1);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coral.shoot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return coralDetector.isGamePieceClose;
  }
}
