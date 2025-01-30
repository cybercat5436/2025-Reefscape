// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import javax.sound.sampled.SourceDataLine;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.Timer;


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
  private double kP = 0.2;
  private Timer timer = new Timer();
  private double robotY;
  private double robotX;
  private double maxSpeed = 3;
  private double timeThreshold = 1.0;
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
  private boolean isAligned;
  private boolean isTimedOut;

  public AutoAlign(CommandSwerveDrivetrain commandSwerveDrivetrain, LimeLight limeLight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(commandSwerveDrivetrain);
    this.commandSwerveDrivetrain = commandSwerveDrivetrain;
    this.limelight = limeLight;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotX = limelight.getVisionArea();
    robotY = limelight.getVisionTargetHorizontalError();

    xSpeed = robotX * kP * maxSpeed;
    ySpeed = robotY * kP * maxSpeed;
    commandSwerveDrivetrain.applyRequest(() ->
    drive.withVelocityX(0) // Drive forward with negative Y (forward)
        .withVelocityY(ySpeed));
    
    

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
