// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DetectReefWithCANrange extends Command {
  private final CANrange reefDetector;
  private Elevator elevator;
  private double higherdistanceThresholdL4 = .5;
  private double lowerdistanceThresholdL4 = .4;
  private boolean readyToShoot;

  /** Creates a new DetectReefWithCANrange. */
  public DetectReefWithCANrange(Elevator elevator) {
    reefDetector = new CANrange(18 );
    this.elevator = new Elevator();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    readyToShoot = false;
    SmartDashboard.putBoolean("Ready to Shoot", readyToShoot);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if ((elevator.getEncoderValue() < 6) && (elevator.getEncoderValue() > 5)) {
        System.out.println("*******Encoder is Correct L4*******");
        if (!((reefDetector.getDistance().getValueAsDouble() < higherdistanceThresholdL4) && (reefDetector.getDistance().getValueAsDouble() > lowerdistanceThresholdL4))) {
          System.out.println("*****Incrementing Elevator L4*****");
      } else {
        System.out.println("******In Shooting Position L4******");
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
