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
  private LimeLight limelight;
  private PhotonVision photonVision;
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
  private double distanceOffTag;
  private double driveTime;
  private Timer timer = new Timer();
  private CANdleSystem candleSystem = CANdleSystem.getInstance();
  private ReefController reefController;
    /** Creates a new AutoAlignWithLimelight. */
    public AutoAlignWheelMovements(CommandSwerveDrivetrain commandSwerveDrivetrain, LimeLight limeLight, PhotonVision photonVision, ReefController reefController) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.addRequirements(commandSwerveDrivetrain);
      this.commandSwerveDrivetrain = commandSwerveDrivetrain;
      this.limelight = limeLight;
      this.photonVision = photonVision;
      this.reefController = reefController;


    // this.movingAverage = new MovingAverage(20);
  }
  public ArrayList<Double> wheelConfigurations(){
      ArrayList<Double> returnValue = new ArrayList<>();
      return returnValue;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    //use distance off of AprilTag 
    distanceOffTag = reefController.getTargetRobotPose().getY();
    candleSystem.showYellow();
    candleSystem.showMagenta();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    commandSwerveDrivetrain.setControl(
      robotCentricDrive
      .withVelocityX(xSpeed)
      );   
  }
  



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= driveTime);
  }
}
