// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WheelMovementsTest extends Command {
  private Timer timer = new Timer();
  private boolean isTimedOut;
  private double timerThreshold = .5;
  private double speed;
  private Pose2d startPosition;
  private Pose2d endPosition;
  private CommandSwerveDrivetrain commandSwerveDrivetrain;
  private SwerveRequest.RobotCentric robotCentricDrive;
 

  /** Creates a new WheelMovementsTest. */
  public WheelMovementsTest(CommandSwerveDrivetrain commandSwerveDrivetrain, double speed, SwerveRequest.RobotCentric robotCentricDrive, Pose2d startPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.robotCentricDrive = robotCentricDrive;
    this.speed = speed;
    this.startPosition = startPosition;
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
    commandSwerveDrivetrain.setControl(robotCentricDrive.withVelocityX(speed));
    System.out.println("Driving Forward");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    commandSwerveDrivetrain.setControl(robotCentricDrive.withVelocityX(0));
    endPosition = commandSwerveDrivetrain.getState().Pose;
    System.out.println("Stopped Driving: Drove for "+timerThreshold+" seconds at power "+speed+" and moved "+ startPosition.getTranslation().getDistance(endPosition.getTranslation()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    isTimedOut = timer.get() > timerThreshold;
    return isTimedOut;
  }
}
