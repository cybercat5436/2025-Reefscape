// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveForward extends Command {
  private Timer timer = new Timer();
  private boolean isTimedOut;
  private double timerThreshold = 1;
  private double speed;

  private CommandSwerveDrivetrain commandSwerveDrivetrain;
  private SwerveRequest.RobotCentric robotCentricDrive;
 

  /** Creates a new DriveForward. */
  public DriveForward(CommandSwerveDrivetrain commandSwerveDrivetrain, double speed, SwerveRequest.RobotCentric robotCentricDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.robotCentricDrive = robotCentricDrive;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    commandSwerveDrivetrain.setControl(robotCentricDrive.withVelocityX(0.3));
    System.out.println("Driving Forward");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    commandSwerveDrivetrain.setControl(robotCentricDrive.withVelocityX(0));
    System.out.println("Stopped Driving");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    isTimedOut = timer.get() > timerThreshold;
    return isTimedOut;
  }
}
