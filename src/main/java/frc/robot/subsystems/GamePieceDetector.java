// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
 

public class GamePieceDetector extends SubsystemBase {
  /** Creates a new CoralSensor. */ 

private boolean isCoralClose;
private double signalStrengthThreshhold;
private CANrange sensorUsed;

public enum Sensors{
  coral,
  algae,
}
  public GamePieceDetector(double signalStrength, Sensors sensor) {
    CANrangeConfiguration config = new CANrangeConfiguration();
    signalStrengthThreshhold = signalStrength;
    
    if (sensor == Sensors.coral){
      sensorUsed = new CANrange(49);

    } else if (sensor == Sensors.algae){
      sensorUsed = new CANrange(45);
    }
    sensorUsed.clearStickyFaults();
    sensorUsed.getConfigurator().apply(config);
    
    
  }
   
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (sensorUsed.getSignalStrength().getValueAsDouble() >=  signalStrengthThreshhold) {
      isCoralClose = true;
    
    } else {
      isCoralClose = false;
    }
    System.out.println(isCoralClose + " signal: " + sensorUsed.getSignalStrength().getValueAsDouble());
    
  }
}
