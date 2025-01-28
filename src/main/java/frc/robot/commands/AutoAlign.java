// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  /** Creates a new AutoAlign. */
  private CommandSwerveDrivetrain commandSwerveDrivetrain;
  private LimeLight limelight;
  private double ySpeed = 0.0;
  private double xSpeed = 0.0;
  private double turningSpeed = 0.0;
  double yError;
  private double targetTx = 1;
  private double horizontalThreshold = 0.5;
  private double kPHorizontal = 1;
  

  public AutoAlign(CommandSwerveDrivetrain commandSwerveDrivetrain, LimeLight limeLight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(commandSwerveDrivetrain);
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.limelight = limeLight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limelight.getVisionTargetHorizontalError();
    System.out.println("Limelight Tx value" + limelight.getVisionTargetHorizontalError());
    yError = limelight.getVisionTargetHorizontalError() - targetTx;
    ySpeed = yError;
    System.out.println("y speed" + ySpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
