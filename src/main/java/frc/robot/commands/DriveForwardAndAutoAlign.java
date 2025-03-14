// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ReefController;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveForwardAndAutoAlign extends Command {
  private Timer timer = new Timer();
  private boolean isTimedOut;
  private double timerThreshold = 0.5;
  private double speed;
  private LimeLight limeLight;

  private double tY;
  private double targettY;
  private double robotYError;
  private double yErrorCalculated;
  private double ySpeed;
  private double maxSpeed = 1;
  private double kPY = 0.075;
  private double horizontalThreshold = 0.5;
  private boolean isYAligned;
  private CommandSwerveDrivetrain commandSwerveDrivetrain;

    // Use reefController to determine target heading
  private ReefController reefController = ReefController.getInstance();
  private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(5, 0, 0)
            .withTargetDirection(reefController.getTargetRobotPose().getRotation());
 

  /** Creates a new DriveForward. */
  public DriveForwardAndAutoAlign(CommandSwerveDrivetrain commandSwerveDrivetrain, double speed, LimeLight limeLight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.speed = speed;
    this.limeLight = limeLight;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set to color pipeline
    LimelightHelpers.setPipelineIndex(limeLight.limelightName, 0);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 1) Calculate Robotcentric x speed vector
    // 2) Calculate Robotcentric y speed vector
    // 2) Add the two to get drive translation

    // 1) Calculate Robot x speed vector
    // determine the drive direction in field coordinates.  
    Rotation2d driveDirection = reefController.getTargetRobotPose().getRotation();
    
    // Construct a Translation2d with magnitude of drive speed and direction from above
    Translation2d rxTranslation2d = new Translation2d(speed, driveDirection);

    // 2) Calculate Robotcentric y speed vector
    tY = limeLight.getVisionTargetVerticalError();
    // tX = -limelight.getVisionTargetHorizontalError();
    robotYError =  tY - targettY;
    // robotXError = targettX - tX;
    yErrorCalculated = kPY * Math.abs(robotYError);
    // xErrorCalculated = kPX * Math.abs(robotXError);
    ySpeed =  Math.min(maxSpeed, Math.abs(yErrorCalculated)) * Math.signum(robotYError);
    // xSpeed = kPX * Math.min(maxSpeed, Math.abs(robotXError)) * Math.signum(robotXError);
    // movingAverage.putData(xSpeed);
    // determine the drive direction in field coordinates.  
    // This 90 degree rotation aligns positive speed to robot left but expresses the angle relative to field
    Rotation2d ryDirection = reefController.getTargetRobotPose().getRotation().rotateBy(Rotation2d.kCCW_90deg);
      
    // Construct a Translation2d with magnitude of drive speed and direction from above
    Translation2d ryTranslation2d = new Translation2d(ySpeed, ryDirection);

    Translation2d driveTranslation2d = rxTranslation2d.plus(ryTranslation2d);

    // Convert to field-centric coordinates by taking vector components
    commandSwerveDrivetrain.setControl(
      fieldCentricFacingAngle
      .withTargetDirection(driveDirection)    // heading and drive direction are same
      .withVelocityY(driveTranslation2d.getY())
      .withVelocityX(driveTranslation2d.getX())
      );
    
    System.out.println("Driving Forward");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    commandSwerveDrivetrain.setControl(fieldCentricFacingAngle
      .withVelocityX(0)
      .withVelocityY(0.0)
      .withTargetDirection(commandSwerveDrivetrain.getState().Pose.getRotation()));

      LimelightHelpers.setPipelineIndex(limeLight.limelightName, 1);
    System.out.println("Stopped Driving");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    isYAligned = Math.abs(robotYError) < horizontalThreshold;
    isTimedOut = timer.get() > timerThreshold;
    return isTimedOut && isYAligned;
  }
}
