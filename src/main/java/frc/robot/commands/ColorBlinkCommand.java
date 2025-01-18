// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.CANdleSystem.AvailableColors;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ColorBlinkCommand extends Command {
  /** Creates a new ColorBlinkCommand. */
  private final AvailableColors color;
  private final CANdleSystem candleSystem;
  private LightState lightState = LightState.Off;
  private Timer timer = new Timer();
  private int numBlinks;
  
  private enum LightState{
    On,
    Off
  }


  public ColorBlinkCommand(AvailableColors color, CANdleSystem candleSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.color = color;
    this.candleSystem = candleSystem;
    addRequirements(this.candleSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initialized ColorBlinkCommand");
    candleSystem.turnOffColors();
    lightState = LightState.On;
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (lightState == LightState.On){
      candleSystem.flashColor(color);
      if (timer.get() > 0.5){
        lightState = LightState.Off;
        System.out.println("Lights turned off");
        timer.reset();
        numBlinks++;
      }
    } else{
      candleSystem.turnOffColors();
      if (timer.get() > 0.25){
        lightState = LightState.On;
        System.out.println("Lights turned on");
        timer.reset();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    candleSystem.turnOffColors();
    System.out.println("Ended...............");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return numBlinks > 3;
  }
}
