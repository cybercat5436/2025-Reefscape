// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.photo.Photo;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.ApplyChassisSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.PhotonVision;
import edu.wpi.first.wpilibj.Timer;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignWithLimelight extends Command {
  private CommandSwerveDrivetrain commandSwerveDrivetrain;
  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private LimeLight limelight;
  private PhotonVision photonVision;
  // private MovingAverage movingAverage;
  private double tX;
  private double targettA = 1.4;
  private double tXError;
  private double tY;
  private double tA;
  private double targettY = -4;
  private double tYError;
  private double kPX = 0.2;
  private double kPY = 0.2;
  private double kPA = 0.2;
  private double ySpeed;
  private double robotYError;
  private double xSpeed;
  private double robotXError;
  private double maxSpeed = 1;
  private Timer timer = new Timer();
  private boolean isYAligned;
  private boolean isXAligned;
  private boolean isTimedOut;
  private double horizontalThreshold = 0.2;
  private double verticalThreshold = 0.2;
  private double timeThreshold = 1;
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

    timer.reset();
    timer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tY = -limelight.getVisionTargetVerticalError();
    tA = limelight.getVisionArea();
    robotYError = targettY - tY;
    robotXError = targettA - tA;
    ySpeed = kPY * Math.min(maxSpeed, Math.abs(robotYError)) * Math.signum(robotYError);
    xSpeed = kPA * Math.min(maxSpeed, Math.abs(robotXError)) * Math.signum(robotXError);
    // movingAverage.putData(xSpeed);
    System.out.println("tX "+tX + "xSpeed " + xSpeed);
    commandSwerveDrivetrain.setControl(
      robotCentricDrive
      .withVelocityY(ySpeed)
      // .withVelocityX(xSpeed)
      );
      System.out.println("******Robot x Error******" + robotXError);
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    LimelightHelpers.setPipelineIndex(limelight.limelightName, 0);
    if(isYAligned && isXAligned) {
      System.out.println("Robot is Aligned");
    }else if(isTimedOut){
      System.out.println("Robot time up");
    }else if(isXAligned) {
      System.out.println("Robot X Aligned");
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
    // return isTimedOut || isYAligned && isXAligned;
    // return isXAligned;
    return isYAligned;

  }
}
