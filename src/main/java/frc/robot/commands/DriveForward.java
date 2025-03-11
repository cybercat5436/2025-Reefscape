// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ReefController;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveForward extends Command {
  private Timer timer = new Timer();
  private boolean isTimedOut;
  private double timerThreshold = 1;
  private double speed;
  private boolean useLockedHeading = false;

  private CommandSwerveDrivetrain commandSwerveDrivetrain;
  private SwerveRequest.RobotCentric robotCentricDrive;

    // Use reefController to determine target heading
  private ReefController reefController = ReefController.getInstance();
  private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(5, 0, 0)
            .withTargetDirection(reefController.getTargetRobotPose().getRotation());
 

  /** Creates a new DriveForward. */
  public DriveForward(CommandSwerveDrivetrain commandSwerveDrivetrain, double speed, SwerveRequest.RobotCentric robotCentricDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.robotCentricDrive = robotCentricDrive;
    this.speed = speed;
  }

  public DriveForward(CommandSwerveDrivetrain commandSwerveDrivetrain, double speed){
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.speed = speed;
    this.useLockedHeading = true;
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
    if(!useLockedHeading){
      commandSwerveDrivetrain.setControl(robotCentricDrive.withVelocityX(0.3));
    } else{
      // determine the drive direction in field coordinates.  
      Rotation2d driveDirection = reefController.getTargetRobotPose().getRotation();
      
      // Construct a Translation2d with magnitude of drive speed and direction from above
      Translation2d driveTranslation2d = new Translation2d(speed, driveDirection);

      // Convert to field-centric coordinates by taking vector components
      commandSwerveDrivetrain.setControl(
        fieldCentricFacingAngle
        .withTargetDirection(driveDirection)    // heading and drive direction are same
        .withVelocityY(driveTranslation2d.getY())
        .withVelocityX(driveTranslation2d.getX())
        );
    }

    System.out.println("Driving Forward");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!useLockedHeading){
      commandSwerveDrivetrain.setControl(robotCentricDrive.withVelocityX(0));
    }else{
      commandSwerveDrivetrain.setControl(fieldCentricFacingAngle
        .withVelocityX(0)
        .withVelocityY(0.0)
        .withTargetDirection(commandSwerveDrivetrain.getState().Pose.getRotation()));
    }
    System.out.println("Stopped Driving");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    isTimedOut = timer.get() > timerThreshold;
    return isTimedOut;
  }
}
