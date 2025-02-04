// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.HashMap;

import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import javax.sound.sampled.SourceDataLine;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  /** Creates a new AutoAlign. */
  private CommandSwerveDrivetrain commandSwerveDrivetrain;
  private LimeLight limelight;
  private double ySpeed = 0.0;
  private double xSpeed = 0.0;
  private double turningSpeed = 0.0;
  double yError;
  private double targetRobotY;
  private double targetRobotX;
  private double horizontalThreshold = 0.2;
  private double VerticalThreshold = 13;
  private Timer timer = new Timer();
  private double robotYError;
  private double robotXError;
  private double maxSpeed = 1;
  private double timeThreshold = 1.0;
  private double MaxAngularRate = 0.75;
  private double rotationRate;
  private double rotationAngle;
  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private boolean isYAligned;
  private boolean isXAligned;
  private boolean isTimedOut;
  private double kPX = 1;
  private double kPY = 0.48;
  private double kDY = 0.05;
  private double currentX;
  private double currentY;
  private double previousYError;
  private double robotYErrorChange;
   

  public AutoAlign(CommandSwerveDrivetrain commandSwerveDrivetrain, LimeLight limeLight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(commandSwerveDrivetrain);
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.limelight = limeLight;
    
    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
    SmartDashboard.putData(this);
  }
  
  


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    System.out.println("starting auto align");


    targetRobotY = 0;
    targetRobotX = 13;//Area target
    rotationAngle = limelight.getRotationAngle();
    
    previousYError = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TO-DO Add robot rotation error from limelight
    // Followed limelight example for aiming and ranging
    // rotationRate = kP * Math.min(MaxAngularRate, Math.abs(robotY)) * Math.signum(robotY);
    // robotXError = limelight.getVisionArea();
    currentX = limelight.getVisionArea();
    currentY = limelight.getVisiontX();
    robotYError = targetRobotY - currentY;
    robotYErrorChange = robotYError - previousYError;
    robotXError = currentX ==0 ? 0.0 : targetRobotX - currentX;

    xSpeed = kPX * Math.min(maxSpeed, Math.abs(robotXError)) * Math.signum(robotXError);
    ySpeed = kPY * Math.min(maxSpeed, Math.abs(robotYError)) * Math.signum(robotYError);
    ySpeed += kDY * robotYErrorChange * maxSpeed;
    commandSwerveDrivetrain.setControl(
        robotCentricDrive
        .withVelocityX(xSpeed) 
        .withVelocityY(ySpeed)
        // .withRotationalRate(rotationAngle)
    );
  

    previousYError = robotYError;


    SmartDashboard.putNumber("Auto align YSpeed", ySpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // To-Do May need to switch robot drive from robot centric to field centric, intial setting to fieldcentric 
    //happens in robotcontainer
    timer.stop();
    
    if(isYAligned) {
      System.out.println("Robot Y Aligned");
    }else if(isTimedOut){
      System.out.println("Robot time up");
    }else if(interrupted){
      System.out.println("Robot interupted");
    }else if (isXAligned){
      System.out.println("Robot X Aligned");
    }
      
    
    
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.addDoubleProperty("kP X", () -> this.kPX, (value) -> this.kPX = value);
    builder.addDoubleProperty("kP Y", () -> this.kPY, (value) -> this.kPY = value);
    builder.addDoubleProperty("kD Y", () -> this.kDY, (value) -> this.kDY = value);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // double distanceError = Math.abs(Math.sqrt(Math.pow(robotX,2))+Math.pow(robotY,2));
    double distanceError = Math.abs(robotYError);
    double XDistanceError = Math.abs(robotXError);
    //  isYAligned = distanceError < horizontalThreshold;
     isXAligned = XDistanceError > VerticalThreshold;
    //  isTimedOut = timer.get() > timeThreshold;
    return isXAligned;
    
  }
}
