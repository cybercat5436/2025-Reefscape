// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.fasterxml.jackson.databind.node.BooleanNode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GamePieceDetector;
import frc.robot.subsystems.LimeLight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DetectReefWithCANrange extends Command {
  private final GamePieceDetector reefDetector;
  private Elevator elevator;
  private double higherdistanceThresholdL4 = .5;
  private double lowerdistanceThresholdL4 = .4;
  private boolean readyToShoot;
  private int lockOutCounter;
  private LimeLight limelight;
  private boolean isLockedOut;
  private int lockOutLimit = 15;
  private int numIncrements;
  private boolean isReefSeen;
  private boolean isAbleToShoot;
  private int numOfFailedAttempts;
  private Timer timer = new Timer();
  private boolean reefNoLongerVisible;
  private boolean reefNeverSeen;
  private boolean isTimedOut;
  private boolean hasRunBefore = false;

  /** Creates a new DetectReefWithCANrange. */
  public DetectReefWithCANrange(Elevator elevator, GamePieceDetector reefDetector) {
    this.reefDetector = reefDetector;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    if(elevator.atTargetHeight()){
    
    }
    

    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
    SmartDashboard.putData(this);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isLockedOut = false;
    isReefSeen = false;
    lockOutCounter = 0;
    numIncrements = 0;
    numOfFailedAttempts = 0;
    timer.reset();
    timer.start();

    reefDetector.setConfiguration(elevator.elevatorLevel);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!elevator.atTargetHeight()) return;
    if(hasRunBefore) return;
    lockOutCounter++;
    if(lockOutCounter >= lockOutLimit) isLockedOut = false;
    
    System.out.println("*******Encoder is Correct*******"); 
    
    // if (!((reefDetector.getDistance().getValueAsDouble() < higherdistanceThresholdL4)
    //   && (reefDetector.getDistance().getValueAsDouble() > lowerdistanceThresholdL4))) {

    //  trying to find when the CANrange does not detect reef.
    // if we see the reef increment elevator height.
    isReefSeen = reefDetector.isGamePieceClose;
    if(isReefSeen && !isLockedOut){
      isLockedOut = true;
      lockOutCounter = 0;
      System.out.println("*****Incrementing Elevator*****"); 
      elevator.incrementHeightAdjustment();
      numIncrements++;
    }
    
    if(!isReefSeen){
      numOfFailedAttempts++;
    }else{
      numOfFailedAttempts = 0;
    }

    
  }
        
  // }
    
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
   builder.addIntegerProperty("lock out count", () -> this.lockOutCounter, null);
   builder.addBooleanProperty("Reef is Seen", () -> this.isReefSeen, null);
   builder.addBooleanProperty("is Locked Out", () -> this.isLockedOut, null);
   builder.addDoubleProperty("Number of Failed Attempts", () -> this.numOfFailedAttempts, null);
   builder.addIntegerProperty("Number of Increments", () -> this.numIncrements, null);
   builder.addBooleanProperty("Reef no longer visible", () -> this.reefNoLongerVisible, null);
   builder.addBooleanProperty("Reef never  visible", () -> this.reefNeverSeen, null);
   builder.addBooleanProperty("is Timed Out", () -> this.isTimedOut, null);
   builder.addBooleanProperty("Target Height", () -> elevator.atTargetHeight(), null);



  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hasRunBefore = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    reefNoLongerVisible = !isReefSeen && numIncrements > 0;
    isAbleToShoot = reefNoLongerVisible && !isLockedOut;
    if(isAbleToShoot) {
      elevator.decrementHeightAdjustment();
      System.out.println(" #$^&*()$#3^$is able to shoot*&^%^#&$#$#$)");
    }
    reefNeverSeen = numOfFailedAttempts > 20;
    if(reefNeverSeen) {
      System.out.println("##$%^&*$#@Reef was never seen**&^%&*(&^%$)");
    }
    // isTimedOut = timer.get() > 3;
    isTimedOut = false;
    // return false;
    return elevator.atTargetHeight() && (reefNeverSeen || isAbleToShoot || isTimedOut || hasRunBefore);
  }
}
