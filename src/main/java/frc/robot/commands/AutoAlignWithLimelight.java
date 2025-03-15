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
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.PhotonVision;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
  private double targettY = 0;
  private double tYError;
  private double kPX = 0.2;
  private double kPY = 0.075;
  private double kPA = 0.2;
  private double kIY = 0.48;
  private double ySpeed;
  private double robotYError;
  private double xSpeed;
  private double robotXError;
  private double maxSpeed = 1;
  private Timer timer = new Timer();
  private boolean isYAligned;
  private boolean isXAligned;
  private boolean isTimedOut;
  private double horizontalThreshold = 0.5;
  private double verticalThreshold = 0.5;
  private double timeThreshold = 1;
  private double yErrorCalculated;
  private double xErrorCalculated;
  private double intergratedError = 0;
  private int isCorrect = 0;
  private CANdleSystem candleSystem = CANdleSystem.getInstance();
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

    candleSystem.showYellow();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tY = limelight.getVisionTargetVerticalError();
    // tX = -limelight.getVisionTargetHorizontalError();
    robotYError =  tY - targettY;
    // robotXError = targettX - tX;
    intergratedError += robotXError * .020;
    yErrorCalculated = kPY * Math.abs(robotYError);
    // xErrorCalculated = kPX * Math.abs(robotXError);
    ySpeed =  Math.min(maxSpeed, Math.abs(yErrorCalculated))* Math.signum(robotYError);
    ySpeed += intergratedError * kIY;
    // xSpeed = kPX * Math.min(maxSpeed, Math.abs(robotXError)) * Math.signum(robotXError);
    // movingAverage.putData(xSpeed);
    // System.out.println("tX "+tX + "xSpeed " + xSpeed);
    commandSwerveDrivetrain.setControl(
      robotCentricDrive
      .withVelocityY(ySpeed)
      // .withVelocityX(xSpeed)
      );
      System.out.println("******Robot x Error******" + robotXError);
      SmartDashboard.putNumber("Successes for AutoAlign", isCorrect);
      SmartDashboard.putNumber("yError", robotYError);
      SmartDashboard.putNumber("Intergrated Error", + intergratedError);
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    LimelightHelpers.setPipelineIndex(limelight.limelightName, 1);
    if(isYAligned && isXAligned) {
      System.out.println("************Robot is Aligned*********");
      candleSystem.setIsAutoAligned(true);
    }else if(isTimedOut){
      System.out.println("Robot time up");
    }else if(isXAligned) {
      System.out.println("Robot X Aligned");
    }else if(isYAligned) {
      System.out.println("**********Robot Y Aligned************"); 
    }
    
    if(isCorrect > 3) {
      System.out.println("*&^%#$@#^*Alignment is correct***!@#$%^&**&^?/%$");
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
    if(LimelightHelpers.getTV(limelight.limelightName)) {
      isYAligned = YDistanceError < horizontalThreshold && (intergratedError < 10);
    }else{
      isYAligned = false;
    }
    // isXAligned = XDistanceError < verticalThreshold;
    isTimedOut = timer.get() > timeThreshold;
    if(isYAligned) {
      isCorrect++;
    }else {
      isCorrect = 0;
    }
    
    
    // return isTimedOut || isYAligned && isXAligned;
    // return isXAligned;
    return isCorrect > 3;

  }
}
