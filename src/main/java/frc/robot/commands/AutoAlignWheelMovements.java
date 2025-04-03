// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import org.opencv.photo.Photo;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.ApplyChassisSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GamePieceDetector;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.ReefController;
import frc.robot.subsystems.PhotonVision;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlignWheelMovements extends Command {
  private CommandSwerveDrivetrain commandSwerveDrivetrain;
  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private LimeLight limeLight;
  // private MovingAverage movingAverage;
  private double tX;
  private double targettA = 1.4;
  private double tXError;
  private double tY;
  private double tA;
  private double targettY = 1.3;
  private double tYError;
  private double kPX = 0.2;
  private double kPY = 0.075;
  private double kPA = 0.2;
  private double kIY = 0.18;
  private double xSpeed;
  private double maxSpeed = 1;
  private double distanceOffReef;
  private double driveTime;
  private Timer timer = new Timer();
  private CANdleSystem candleSystem = CANdleSystem.getInstance();
  private ReefController reefController;
    /** Creates a new AutoAlignWheelMovements. */
    public AutoAlignWheelMovements(CommandSwerveDrivetrain commandSwerveDrivetrain, LimeLight limeLight, ReefController reefController) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.addRequirements(commandSwerveDrivetrain);
      this.commandSwerveDrivetrain = commandSwerveDrivetrain;
      this.limeLight = limeLight;
      this.reefController = reefController;
      

    // this.movingAverage = new MovingAverage(20);
  }
  public double calculateTime(double meters){
      //we will fix a power and then calculate how far to move (in meters)
      //power is 0.65, this is the formula from google sheets with R^2=.98
      double time = 0.625*meters-0.0208;
      if (time>0) {
        return time;
      }
      return 0;
  }
  public double cameraToMeters(LimeLight limelight) {
    double ty = limelight.getVisionTargetVerticalError();
    return (0.775)*(Math.tan(Math.toRadians(ty)) - Math.tan(Math.toRadians(1.6)));
    //get tx and then angle offset
    //ty at current alignment is 1.6 degrees to the left, or +1.6
    //we want to move the robot so that our angle between is 1.6 degrees, we are given the distance from bot to reef as 77.5 degrees.
    //use limelight-right
    //keep in mind the directed offset!
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //LimelightHelpers.setPipelineIndex(limeLight.limelightName,3);
    timer.reset();
    //use distance off of AprilTag 
    candleSystem.showYellow();
    candleSystem.showMagenta();
    timer.start();
    distanceOffReef = cameraToMeters(this.limeLight);
    this.driveTime = calculateTime(Math.abs(distanceOffReef));
    System.out.println("Auto Align Wheel Movements: driving for "+this.driveTime);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimelightHelpers.setPipelineIndex(limeLight.limelightName,3);
    if (distanceOffReef > 0) {
    commandSwerveDrivetrain.setControl(
      robotCentricDrive
      .withVelocityY(0.65)
      );
    } else {
      commandSwerveDrivetrain.setControl(
        robotCentricDrive
        .withVelocityY(-0.65)
        );
    }
  }
  
//6.286, 4.206


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    commandSwerveDrivetrain.setControl(
      robotCentricDrive
      .withVelocityY(0)
      .withVelocityX(0));
    System.out.println("********************* Finished aligning: drove for"+distanceOffReef+"meters");
    LimelightHelpers.setPipelineIndex(limeLight.limelightName,1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= driveTime);
  }
}
