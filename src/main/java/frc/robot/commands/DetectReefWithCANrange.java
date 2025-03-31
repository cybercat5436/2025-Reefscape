// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GamePieceDetector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DetectReefWithCANrange extends Command {
  private final GamePieceDetector reefDetector;
  private Elevator elevator;
  private boolean isReefSeen;
  private boolean isHeightAdjusted;
  private boolean hasCommandExecuted = false;

  /** Creates a new DetectReefWithCANrange. */
  public DetectReefWithCANrange(Elevator elevator, GamePieceDetector reefDetector) {
    this.reefDetector = reefDetector;
    this.elevator = elevator;
    hasCommandExecuted = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);

    SendableRegistry.addLW(this, this.getClass().getSimpleName(), this.getClass().getSimpleName());
    SmartDashboard.putData(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isHeightAdjusted = false;
    isReefSeen = true;
    // reefDetector.setConfiguration(elevator.elevatorLevel);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // guard against execution if command has already run
    if (hasCommandExecuted) return;

    isReefSeen = reefDetector.isGamePieceClose;

    // if reef is visible, move the elevator up slowly until it is no longer seen
    if(isReefSeen){
      elevator.moveUpSlowlyUntilReefNotSeen();
      isHeightAdjusted = true;
    }
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(hasCommandExecuted) System.out.println("~~~~~~   Command has already been executed, skipping   ~~~~~~~~");
    if(isReefSeen) System.out.println("~~~~~~   Reef is visible   ~~~~~~~~");
    if(!isReefSeen) System.out.println("~~~~~~   Reef is not visible   ~~~~~~~~");

    // return false;
    return !isReefSeen || hasCommandExecuted;
  }
        
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("~~~~~~~~~~  Running end from DetectReefWithCANrange  ~~~~~~~~~~~~");
    
    // if interrupted, don't apply clean-up commands and don't set as executed
    if(interrupted) {
      System.out.println("~~~~~~~~~~~  Inside command and INTERRUPTED   ~~~~~~~~~~~");
      return;
    }
    
    if(isHeightAdjusted){
      // command executed for the first time and is ready to score
      // if the height was adjusted then transfer the new height to elevator
      double newValue = elevator.holdCurrentPosition();
      elevator.transferPositionToLevel(newValue);
      System.out.println("~~~~~~~~~~~  New height set for level " + elevator.elevatorLevel + " new Position: " + newValue + "   ~~~~~~~~~~~");
    }

    // set this flag to avoid execution next time command is scheduled
    hasCommandExecuted = true;
  }


  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
    super.initSendable(builder);
    builder.addBooleanProperty("Target Height", () -> elevator.atTargetHeight(), null);
    builder.addBooleanProperty("hasCommandExecuted", () -> this.hasCommandExecuted, null);
    builder.addBooleanProperty("Reef is Seen", () -> this.isReefSeen, null);
    builder.addBooleanProperty("Reef no longer visible", () -> this.isHeightAdjusted, null);
  }
}
