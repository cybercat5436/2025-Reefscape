// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DetectReefWithCANrange extends Command {
  private final CANrange reefDetector;
  private Elevator elevator;
  private double higherdistanceThreshold = .46;
  private double lowerdistanceThreshold = .42;
  private boolean readyToShoot = false;
  /** Creates a new DetectReefWithCANrange. */
  public DetectReefWithCANrange() {
    reefDetector = new CANrange(0);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if ((elevator.getEncoderValue() < 6) && (elevator.getEncoderValue() > 5.8)) {
         if ((reefDetector.getDistance().getValueAsDouble() < higherdistanceThreshold) && (reefDetector.getDistance().getValueAsDouble() > lowerdistanceThreshold)) {
        elevator.incrementHeightAdjustment();
      } else {
        readyToShoot = true;
      }
      }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return readyToShoot;
  }
}
