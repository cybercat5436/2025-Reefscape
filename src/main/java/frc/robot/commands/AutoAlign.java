// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.HashMap;

import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;
import java.util.List;

import javax.sound.sampled.SourceDataLine;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.units.measure.AngularVelocity;
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
  private double targetTx = 1;
  private double horizontalThreshold = 0.2;
  private double kP = 0.3;
  private Timer timer = new Timer();
  private double robotY;
  private double robotX;
  private double maxSpeed = 3;
  private double timeThreshold = 1.0;
  private double MaxAngularRate = 0.75;
  private double rotationRate = 0.0;
  private double rotationAngle;
  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private boolean isAligned;
  private boolean isTimedOut;
   

  public AutoAlign(CommandSwerveDrivetrain commandSwerveDrivetrain, LimeLight limeLight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(commandSwerveDrivetrain);
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.limelight = limeLight;
    
    // HashMap<Double, Double> RotationTableLookUpAngles = new HashMap<>();
    // RotationTableLookUpAngles.put(6.0,300.0);
    // RotationTableLookUpAngles.put(7.0,0.0);
    // RotationTableLookUpAngles.put(8.0, 60.0);
    // RotationTableLookUpAngles.put(9.0,120.0);
    // RotationTableLookUpAngles.put(10.0, 180.0);
    // RotationTableLookUpAngles.put(11.0,240.0);
    // RotationTableLookUpAngles.put(17.0,240.0);
    // RotationTableLookUpAngles.put(18.0,180.0);
    // RotationTableLookUpAngles.put(19.0, 120.0);
    // RotationTableLookUpAngles.put(20.0, 60.0);
    // RotationTableLookUpAngles.put(21.0,0.0);
    // RotationTableLookUpAngles.put(22.0,300.0);

    HashMap<Integer, Double> RotationTableLookUpAngles = new HashMap<>();
    RotationTableLookUpAngles.put(6,300.0);
    RotationTableLookUpAngles.put(7,0.0);
    RotationTableLookUpAngles.put(8, 60.0);
    RotationTableLookUpAngles.put(9,120.0);
    RotationTableLookUpAngles.put(10, 180.0);
    RotationTableLookUpAngles.put(11,240.0);
    RotationTableLookUpAngles.put(17,240.0);
    RotationTableLookUpAngles.put(18,180.0);
    RotationTableLookUpAngles.put(19, 120.0);
    RotationTableLookUpAngles.put(20, 60.0);
    RotationTableLookUpAngles.put(21,0.0);
    RotationTableLookUpAngles.put(22,300.0);
  
    rotationAngle = RotationTableLookUpAngles.get(limeLight.getAprilTagId());
    
    

  }
  
  


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    System.out.println("starting auto align");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotX = limelight.getVisionArea();
    robotY = -limelight.getVisionTargetHorizontalError();
    // TO-DO Add robot rotation error from limelight
    // Followed limelight example for aiming and ranging
    rotationRate = robotY * kP * MaxAngularRate;
    xSpeed = robotX * kP * maxSpeed;
    ySpeed = robotY * kP * maxSpeed;
    commandSwerveDrivetrain.applyRequest(() ->
      // drive.withVelocityY(ySpeed));
        robotCentricDrive
        .withVelocityX(xSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(ySpeed)
        .withRotationalRate(rotationRate));

    


    SmartDashboard.putNumber("Auto align YSpeed", ySpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // To-Do May need to switch robot drive from robot centric to field centric, intial setting to fieldcentric 
    //happens in robotcontainer
    timer.stop();
    
    if(isAligned) {
      System.out.println("Robot Aligned");
    }else if(isTimedOut){
      System.out.println("Robot time up");
    }else if(interrupted){
      System.out.println("Robot interupted");
    }
      
    
    
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // double distanceError = Math.abs(Math.sqrt(Math.pow(robotX,2))+Math.pow(robotY,2));
    double distanceError = Math.abs(robotY);
     isAligned = distanceError < horizontalThreshold;
     isTimedOut = timer.get() > timeThreshold;
    return isAligned || isTimedOut;
    
  }
}
