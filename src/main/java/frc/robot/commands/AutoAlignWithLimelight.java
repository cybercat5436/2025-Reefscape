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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.ReefController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignWithLimelight extends Command {
  private CommandSwerveDrivetrain commandSwerveDrivetrain;

  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  // Use reefController to determine target heading
  private ReefController reefController = ReefController.getInstance();
  private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(5, 0, 0)
            .withTargetDirection(reefController.getTargetRobotPose().getRotation());

  private LimeLight limelight;
  private PhotonVision photonVision;
  // private MovingAverage movingAverage;
  private double tX;
  private double targettA = 1.4;
  private double tXError;
  private double tY;
  private double tA;
  private double targettY = 0;
  private double tYError;
  private double kPX = 0.2;
  private double kPY = 0.075;
  private double kPA = 0.2;
  private double ySpeed;
  private double robotYError;
  private double xSpeed;
  private double robotXError;
  private double maxSpeed = 1;
  private Timer timer = new Timer();
  private boolean isYAligned;
  private boolean isXAligned;
  private boolean isHeadingAligned = false;
  private boolean isTimedOut;
  private double horizontalThreshold = 0.5;
  private double verticalThreshold = 0.5;
  private double rotationDegreesThreshold = 0.5;
  private double timeThreshold = 1;
  private double yErrorCalculated;
  private double xErrorCalculated;
  private int isCorrect = 0;
  private boolean useLockedHeading = false;

  /** Creates a new AutoAlignWithLimelight. */
  public AutoAlignWithLimelight(CommandSwerveDrivetrain commandSwerveDrivetrain, LimeLight limeLight, PhotonVision photonVision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(commandSwerveDrivetrain);
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.limelight = limeLight;
    this.photonVision = photonVision;
    // this.movingAverage = new MovingAverage(20);

  }

    /** Creates a new AutoAlignWithLimelight. */
    public AutoAlignWithLimelight(CommandSwerveDrivetrain commandSwerveDrivetrain, LimeLight limeLight, PhotonVision photonVision, boolean useLockedHeading) {
      // Use addRequirements() here to declare subsystem dependencies.
      this(commandSwerveDrivetrain, limeLight, photonVision);
      this.useLockedHeading = useLockedHeading;
  
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex(limelight.limelightName, 0);

    timer.reset();
    timer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tY = limelight.getVisionTargetVerticalError();
    // tX = -limelight.getVisionTargetHorizontalError();
    robotYError =  tY - targettY;
    // robotXError = targettX - tX;
    yErrorCalculated = kPY * Math.abs(robotYError);
    // xErrorCalculated = kPX * Math.abs(robotXError);
    ySpeed =  Math.min(maxSpeed, Math.abs(yErrorCalculated)) * Math.signum(robotYError);
    // xSpeed = kPX * Math.min(maxSpeed, Math.abs(robotXError)) * Math.signum(robotXError);
    // movingAverage.putData(xSpeed);

    
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

    // System.out.println("~~~~ tY: "+tY + "  -- ySpeed: " + ySpeed + "~~~~~~~~");
    System.out.println("~~~~~~~ Chassis Speeds: " + commandSwerveDrivetrain.getState().Speeds.toString());
    System.out.println("~~~~~~~ Target Heading: " + fieldCentricFacingAngle.HeadingController.getSetpoint() + "   Actual Heading: " + commandSwerveDrivetrain.getState().Pose.getRotation().getDegrees());

  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    LimelightHelpers.setPipelineIndex(limelight.limelightName, 1);
    
    if(isCorrect > 3) {
      System.out.println("*&^%#$@#^*     Alignment is sustained        ***!@#$%^&**&^?/%$");
    } else if(isYAligned && isXAligned) {
      System.out.println("************   Robot is Aligned     *********");
    }else if(isTimedOut){
      System.out.println("~~~~~~~   Robot time up  ~~~~~~~~~~~~");
    }else if(isXAligned) {
      System.out.println("Robot X Aligned");
    }else if(isYAligned) {
      System.out.println("**********Robot Y Aligned************");
    } else if(interrupted){
      System.out.println("~~~~  Auto align exited because interrupted   ~~~~~");
    }

    commandSwerveDrivetrain.setControl(
      robotCentricDrive
      .withVelocityY(0)
      .withVelocityX(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double YDistanceError = Math.abs(robotYError);
    double XDistanceError = Math.abs(robotXError);
    isYAligned = YDistanceError < horizontalThreshold;
    isXAligned = XDistanceError < verticalThreshold;
    isTimedOut = timer.get() > timeThreshold;
    
    boolean isAligned = isYAligned;
    
    if (useLockedHeading){
      var rotationError = commandSwerveDrivetrain.getState().Pose.getRotation().minus(ReefController.getInstance().getTargetRobotPose().getRotation());
      isHeadingAligned = Math.abs(rotationError.getDegrees()) < rotationDegreesThreshold;
      isAligned = (isYAligned && isHeadingAligned);
    }
    
    
    if(isAligned) {
      isCorrect++;
    }else {
      isCorrect = 0;
    }
    
    
    // return isTimedOut || isYAligned && isXAligned;
    // return isXAligned;
    return isCorrect > 3;

  }
}
