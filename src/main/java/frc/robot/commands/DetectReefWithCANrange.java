// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  private boolean isLockedOut;
  private int lockOutLimit = 3;
  private int numIncrements;
  private boolean isReefSeen;
  private boolean isReefNotSeen;
  private Timer timer = new Timer();
  private boolean reefNoLongerVisible;
  private boolean reefNeverSeen;
  private boolean isTimedOut;
  private ArrayList<Integer> reefObservations = new ArrayList<>();
  
  // change these two together
  private int limitObservations = 20;    // check minValuesForCofidence is not greater
  private int minValuesForConfidence = 10;

  private double confidenceThreshold = 0.7;
  private boolean hasCommandExecuted = false;
  private boolean isObservationBufferFull = false;

  /** Creates a new DetectReefWithCANrange. */
  public DetectReefWithCANrange(Elevator elevator, GamePieceDetector reefDetector) {
    this.reefDetector = reefDetector;
    this.elevator = elevator;
    hasCommandExecuted = false;
    // Use addRequirements() here to declare subsystem dependencies.
    
    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
    SmartDashboard.putData(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isLockedOut = false;
    isReefSeen = false;
    isReefNotSeen = false;
    lockOutCounter = 0;
    numIncrements = 0;
    timer.reset();
    timer.start();
    reefObservations.clear();
    isObservationBufferFull = false;
    reefDetector.setConfiguration(elevator.elevatorLevel);
  }

  private void addObservation(boolean observation){
    // manage the size of the buffer
    if(reefObservations.size() >= limitObservations) reefObservations.remove(0);
    // add the item
    reefObservations.add(observation ? 1 : 0);
  }

  private double getReefConfidence(){
    // guard if buffer is sparse
    var minLimit = Math.min(minValuesForConfidence, limitObservations);  // just in case entered wrong
    if (reefObservations.size() < minLimit) return -1.0;
    
    // calculate the average of the values in the array
    double confidence = reefObservations.stream().mapToInt(Integer::intValue).average().orElse(-1.0);
    return confidence;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (hasCommandExecuted) return;

    lockOutCounter++;

    if(lockOutCounter >= lockOutLimit) isLockedOut = false;
    
    // add reef observation
    addObservation(reefDetector.isGamePieceClose);
    isObservationBufferFull = (reefObservations.size() >= limitObservations);

    // apply conficence level to reef observation, this is instead of using consecutive readings
    // note that it leaves a buffer zone where both can be false
    var confidence = getReefConfidence();
    isReefSeen = confidence > confidenceThreshold;
    boolean isBufferTooSmall = (confidence == -1.0);
    isReefNotSeen = isBufferTooSmall ? false : (1.0 - confidence) > confidenceThreshold;
    
    //  trying to find when the CANrange does not detect reef.
    // if we see the reef increment elevator height.
    if(isReefSeen && !isLockedOut){
      isLockedOut = true;
      lockOutCounter = 0;
      System.out.println("*****Incrementing Elevator*****"); 
      elevator.incrementHeightAdjustment();
      numIncrements++;
    }
    
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    reefNoLongerVisible = isReefNotSeen && numIncrements > 0;

    if(reefNoLongerVisible) {
      System.out.println(" #$^&*()$#3^$   is able to shoot     *&^%^#&$#$#$)");
    }


    reefNeverSeen = isReefNotSeen && isObservationBufferFull;
    if(reefNeverSeen) {
      System.out.println("##$%^&*$#@      Reef was never seen      **&^%&*(&^%$)");
    }

    if(hasCommandExecuted) System.out.println("~~~~~~   Command has already been executed, skipping   ~~~~~~~~");

    // return false;
    return reefNoLongerVisible || reefNeverSeen || hasCommandExecuted;
  }
        
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("~~~~~~~~~~  Running end from DetectReefWithCANrange  ~~~~~~~~~~~~");
    if(interrupted) System.out.println("~~~~~~~~~~~  Inside command and INTERRUPTED   ~~~~~~~~~~~");

    hasCommandExecuted = true;
  }


  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.addIntegerProperty("lock out count", () -> this.lockOutCounter, null);
    builder.addBooleanProperty("Reef is Seen", () -> this.isReefSeen, null);
    builder.addBooleanProperty("Reef is NOT Seen", () -> this.isReefNotSeen, null);
    builder.addBooleanProperty("is Locked Out", () -> this.isLockedOut, null);
    builder.addIntegerProperty("Number of Increments", () -> this.numIncrements, null);
    builder.addBooleanProperty("Reef no longer visible", () -> this.reefNoLongerVisible, null);
    builder.addBooleanProperty("Reef never  visible", () -> this.reefNeverSeen, null);
    builder.addBooleanProperty("is Timed Out", () -> this.isTimedOut, null);
    builder.addBooleanProperty("Target Height", () -> elevator.atTargetHeight(), null);
    builder.addBooleanProperty("hasCommandExecuted", () -> this.hasCommandExecuted, null);
    builder.addDoubleProperty("Reef Confidence", () -> getReefConfidence(), null);
  }
}
