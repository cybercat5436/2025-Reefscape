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

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.AutoAlignCANdle;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.PhotonVision;
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
  private PhotonVision photonVision;
  private double ySpeed = 0.0;
  private double xSpeed = 0.0;
  private double turningSpeed = 0.0;
  double yError;
  private double targetYaw;
  private double targetArea;
  private double horizontalThreshold = 0.1;
  private double VerticalThreshold = 1;
  private Timer timer = new Timer();
  private double robotYError;
  private double robotXError;
  private double maxSpeed = 1;
  private double timeThreshold = 1.0;
  private double MaxAngularRate = 0.75;
  private double rotationRate;
  private double targetRotationAngle;
  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private boolean isYAligned;
  private boolean isXAligned;
  private boolean isRotationAligned;
  private boolean isTimedOut;
  private double kPX = 0.15;
  private double kPY = 0.1;
  private double kDY = 0;
  private double rotationkP = 0.75;//0.75
  private double currentArea;
  private double currentYaw;
  private double previousYError;
  private double robotYErrorChange;
  private double targetRotation;
  private double rotationError;
  private double currentRotation;
  private double rotationSpeed;


  public AutoAlign(CommandSwerveDrivetrain commandSwerveDrivetrain, LimeLight limeLight, PhotonVision photonVision) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(commandSwerveDrivetrain);
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.limelight = limeLight;
    this.photonVision = photonVision;
    
    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
  
    SmartDashboard.putData(this);
  }
  
  


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex(limelight.limelightName, 0);
    System.out.println("********Change Limelight Pipeline to 0 in autoalign ********");

    timer.reset();
    timer.start();
    System.out.println("starting auto align");
    
    //ID 20
    targetYaw = -19.2;
    targetArea = 9.5;//Area target
    // targetRotation = -120;

    //ID 21
    targetRotation = 180;

    previousYError = 0;

    SmartDashboard.putNumber("Robot Y Error", robotYError);
    SmartDashboard.putNumber("Robot X Error", robotXError);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TO-DO Add robot rotation error from limelight
    // Followed limelight example for aiming and ranging
    // rotationRate = kP * Math.min(MaxAngularRate, Math.abs(robotY)) * Math.signum(robotY);
    // robotXError = limelight.getVisionArea();
    currentRotation =commandSwerveDrivetrain.getState().Pose.getRotation().getDegrees();
    currentArea = photonVision.observedArea;
    currentYaw = photonVision.observedYaw;
    robotYError = targetYaw - currentYaw;
    robotYErrorChange = robotYError - previousYError;
    robotXError = currentArea == 0 ? 0.0 : targetArea - currentArea;
    rotationError = boundAngle(targetRotation - currentRotation);

    

    xSpeed = kPX * Math.min(maxSpeed, Math.abs(robotXError)) * Math.signum(robotXError);
    ySpeed = kPY * Math.min(maxSpeed, Math.abs(robotYError)) * Math.signum(robotYError);
    rotationSpeed = rotationkP * Math.min(MaxAngularRate, Math.abs(rotationError)) * Math.signum(rotationError);
    ySpeed += kDY * robotYErrorChange * maxSpeed;
    commandSwerveDrivetrain.setControl(
        robotCentricDrive
        .withVelocityX(xSpeed) 
        .withVelocityY(ySpeed)
        .withRotationalRate(rotationSpeed)
    );
  

    previousYError = robotYError;

    SmartDashboard.putNumber("Robot rotation pose", commandSwerveDrivetrain.getState().Pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Auto align YSpeed", ySpeed);
    SmartDashboard.putNumber("Robot Y Error", robotYError);
    SmartDashboard.putNumber("Robot X Error", robotXError);
  }

  private double boundAngle(double angleIn) {
    if(angleIn <= -180) {
      angleIn = angleIn + 360;
    }
    if(angleIn > 180) {
      angleIn = angleIn - 360;
    }
    return angleIn;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // To-Do May need to switch robot drive from robot centric to field centric, intial setting to fieldcentric 
    //happens in robotcontainer
    timer.stop();
    
    commandSwerveDrivetrain.setControl(
        robotCentricDrive
        .withVelocityX(0) 
        .withVelocityY(0)
        .withRotationalRate(0)
    );
    LimelightHelpers.setPipelineIndex(limelight.limelightName, 1);
    System.out.println("********Change Limelight Pipeline to 1 in autoalign ********");

    
    if(isYAligned) {
      System.out.println("Robot Y Aligned");
    }else if(isTimedOut){
      System.out.println("Robot time up");
    }else if(interrupted){
      System.out.println("Robot interupted");
    }else if (isXAligned){
      System.out.println("Robot X Aligned");
    }else if(isRotationAligned){
      System.out.println("Rotaion Aligned");
    }
      
    
  }
  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.addDoubleProperty("kP X", () -> this.kPX, (value) -> this.kPX = value);
    builder.addDoubleProperty("kP Y", () -> this.kPY, (value) -> this.kPY = value);
    builder.addDoubleProperty("kD Y", () -> this.kDY, (value) -> this.kDY = value);
    builder.addDoubleProperty("kP Rotation", () -> this.rotationkP, (value) -> this.rotationkP = value);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // double distanceError = Math.abs(Math.sqrt(Math.pow(robotX,2))+Math.pow(robotY,2));
    double YDistanceError = Math.abs(robotYError);
    double XDistanceError = Math.abs(robotXError);
    double RotationError = Math.abs(rotationError);
     isYAligned = YDistanceError < horizontalThreshold;
     isXAligned = XDistanceError < VerticalThreshold;
     isRotationAligned = RotationError < 2;
    //  isTimedOut = timer.get() > timeThreshold;
    //  isYAligned = (YDistanceError < horizontalThreshold) || isYAligned;
    //  isXAligned = (XDistanceError < VerticalThreshold) || isXAligned;
    //  isRotationAligned = (RotationError < 2) || isRotationAligned;
    // if(isYAligned && kPY != 0){
    //   kPY = 0;
    //   System.out.println("KP Y is 0");
    // }
    // if(isXAligned && kPX != 0){
    //   kPX = 0;
    // }
    // if(isRotationAligned && rotationkP != 0){
    //   rotationkP = 0;
    // }

    return  isTimedOut || isXAligned && isYAligned && isRotationAligned;
    
  }
}
