// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignWithSlowMode extends Command {
  private CommandSwerveDrivetrain commandSwerveDrivetrain;
  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private LimeLight limelight;
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
  private double kIY = 0.24; //0.24
  private double ySpeed;
  private double robotYError;
  private double xSpeed;
  private double rotationSpeed;
  private double robotXError;
  private double maxSpeed = 1;
  private Timer timer = new Timer();
  private boolean isYAligned;
  private boolean isXAligned;
  private boolean isTimedOut;
  private double horizontalThreshold = 0.5;
  private double verticalThreshold = 0.5;
  private double timeThreshold = 30;  
  private double yErrorCalculated;
  private double xErrorCalculated;
  private double intergratedError = 0;
  private int isCorrect = 0;
  private CANdleSystem candleSystem = CANdleSystem.getInstance();
  private DoubleSupplier joystickRobotY;
  private DoubleSupplier joystickRobotX;
  private DoubleSupplier joystickRobotRotation;
  private double slowModeSpeed = 0.78;
  private double slowModeRoationSpeed = 1.18;
  /** Creates a new AutoAlignWithLimelight. */
  public AutoAlignWithSlowMode(CommandSwerveDrivetrain commandSwerveDrivetrain, LimeLight limeLight, 
  DoubleSupplier joystickRobotX, DoubleSupplier joystickRobotY, DoubleSupplier joystickRobotRotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(commandSwerveDrivetrain);
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.limelight = limeLight;    this.joystickRobotX = joystickRobotX;
    this.joystickRobotY = joystickRobotY;
    this.joystickRobotRotation = joystickRobotRotation;
    
    // this.movingAverage = new MovingAverage(20);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    candleSystem.showYellow();
    System.out.println("**********enter autoalignWithSlowMode with limelight**********");
    Alliance alliance = DriverStation.getAlliance().isPresent()?DriverStation.getAlliance().get():DriverStation.Alliance.Blue;
    if(alliance == Alliance.Blue) {
      LimelightHelpers.setPipelineIndex(limelight.limelightName, 0);
    }else{
      LimelightHelpers.setPipelineIndex(limelight.limelightName, 2);

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
   
      xSpeed = joystickRobotX.getAsDouble() * slowModeSpeed;
      rotationSpeed = joystickRobotRotation.getAsDouble() * slowModeRoationSpeed;

      tY = limelight.getVisionTargetVerticalError();
      // tX = -limelight.getVisionTargetHorizontalError();
      robotYError =  tY - targettY;
      // robotXError = targettX - tX;
      intergratedError += robotYError * .005;
      yErrorCalculated = kPY * Math.abs(robotYError);
      // xErrorCalculated = kPX * Math.abs(robotXError);
      ySpeed =  Math.min(maxSpeed, Math.abs(yErrorCalculated))* Math.signum(robotYError);
      if(Math.abs(robotYError) < 2){
      ySpeed += intergratedError * kIY;
    
    
    
    
  }
    // xSpeed = kPX * Math.min(maxSpeed, Math.abs(robotXError)) * Math.signum(robotXError);
    // movingAverage.putData(xSpeed);
    // System.out.println("tX "+tX + "xSpeed " + xSpeed);
    commandSwerveDrivetrain.setControl(
      robotCentricDrive
      .withVelocityY(ySpeed)
      .withVelocityX(xSpeed)
      .withRotationalRate(rotationSpeed)
      );
      System.out.println("******Robot Y Error******" + robotYError);
      SmartDashboard.putNumber("Successes for AutoAlign", isCorrect);
      SmartDashboard.putNumber("yError", robotYError);
      SmartDashboard.putNumber("intergrated Y Error", intergratedError);
      SmartDashboard.putBoolean("is target visable", LimelightHelpers.getTV(limelight.limelightName));
      SmartDashboard.putNumber("Auto Align Timer", timer.get());
      SmartDashboard.putNumber("Limelight Pipeline", LimelightHelpers.getCurrentPipelineIndex(limelight.limelightName));
   

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
    CANdleSystem.getInstance().setIsAligned(isCorrect > 3);
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
      isYAligned = YDistanceError < horizontalThreshold && (intergratedError < 6);
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
    return isCorrect > 3 || isTimedOut;

  }
}
