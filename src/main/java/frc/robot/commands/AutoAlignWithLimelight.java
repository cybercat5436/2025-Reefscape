// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.photo.Photo;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.ApplyChassisSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ReefController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignWithLimelight extends Command {
  private CommandSwerveDrivetrain commandSwerveDrivetrain;

  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  // Use reefController to determine target heading
  private ReefController reefController = ReefController.getInstance();
  private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(2, 0, 0)
            .withTargetDirection(reefController.getTargetRobotPose().getRotation());

  private LimeLight limelight;
  private PhotonVision photonVision;
  // private MovingAverage movingAverage;
  private double tX;
  private double targettX = 0;
  private double tXError;
  private double tY;
  private double targettY = -3.2;
  private double tYError;
  private double kPX = 0.2;
  private double kPY = 0.2;
  private double ySpeed;
  private double robotYError;
  private double xSpeed;
  private double robotXError;
  private double maxSpeed = 1;
  private boolean useLockedHeading = true;

  /** Creates a new AutoAlignWithLimelight. */
  public AutoAlignWithLimelight(CommandSwerveDrivetrain commandSwerveDrivetrain, LimeLight limeLight, PhotonVision photonVision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(commandSwerveDrivetrain);
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.limelight = limeLight;
    this.photonVision = photonVision;
    // this.movingAverage = new MovingAverage(20);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex(limelight.limelightName, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // the limelight used is rotate 90deg, 
    // -- positive +ty value is left
    // -- positive +tx value is up

    tY = limelight.getVisionTargetVerticalError();  // the value provided is limelight ty, removed negative sign
    tX = limelight.getVisionTargetHorizontalError();
    
    robotYError = tY - targettY;  // switched order of terms to reverse sign
    robotXError = tX - targettX;
    
    // The speed clipping should occur after multiplying by kPY
    // ySpeed = kPY * Math.min(maxSpeed, Math.abs(robotYError)) * Math.signum(robotYError);
    ySpeed = kPY * Math.abs(robotYError);  //Magnitude of speed
    ySpeed = Math.min(maxSpeed, ySpeed) * Math.signum(robotYError);  // clamped magnitude with sign applied
    
    xSpeed = kPX * Math.abs(robotXError);
    xSpeed = Math.min(maxSpeed, xSpeed) * Math.signum(robotXError);


    ySpeed = 1;
    // movingAverage.putData(xSpeed);
    System.out.println("~~~~ tX "+tX + "xSpeed " + xSpeed + "~~~~~~~~");
    
    if (!useLockedHeading){
      commandSwerveDrivetrain.setControl(
        robotCentricDrive
        .withVelocityY(ySpeed));
    } else{
      // determine the drive direction in field coordinates.  
      // This 90 degree rotation aligns positive speed to robot left but expresses the angle relative to field
      Rotation2d driveDirection = reefController.getTargetRobotPose().getRotation().rotateBy(Rotation2d.kCCW_90deg);
      
      // Construct a Translation2d with magnitude of drive speed and direction from above
      Translation2d driveTranslation2d = new Translation2d(ySpeed, driveDirection);

      commandSwerveDrivetrain.setControl(
        fieldCentricFacingAngle
        .withTargetDirection(reefController.getTargetRobotPose().getRotation())
        .withVelocityY(driveTranslation2d.getY())
        .withVelocityX(driveTranslation2d.getX())
        );


    }

  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LimelightHelpers.setPipelineIndex(limelight.limelightName, 1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
