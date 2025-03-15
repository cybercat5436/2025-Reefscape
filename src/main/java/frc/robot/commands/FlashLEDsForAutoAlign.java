// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.LimeLight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FlashLEDsForAutoAlign extends Command {
  /** Creates a new FlashLEDsForAutoAlign. */
  private Timer timer = new Timer();
  private LimeLight limelight;
  public FlashLEDsForAutoAlign(LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limelight = limelight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    CANdleSystem.getInstance().showGreen();
    LimelightHelpers.setLEDMode_ForceBlink(limelight.getName());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();

    CANdleSystem.getInstance().setIsAligned(false);
    CANdleSystem.getInstance().turnOffColors();
    LimelightHelpers.setLEDMode_ForceOff(limelight.getName());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return timer.get() > 3;
  }
}
