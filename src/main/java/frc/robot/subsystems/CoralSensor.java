// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
 

public class CoralSensor extends SubsystemBase {
  /** Creates a new CoralSensor. */ 
private CANrange canRange = new CANrange(49);
public boolean isCoralClose;
  public CoralSensor() {
    CANrangeConfiguration config = new CANrangeConfiguration();
    canRange.clearStickyFaults();
    canRange.getConfigurator().apply(config);
    
  }
   
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (canRange.getSignalStrength().getValueAsDouble() >=  3000) {
      isCoralClose = true;
    
    } else {
      isCoralClose = false;
    }
    // System.out.println(isCoralClose + " signal: " + canRange.getSignalStrength().getValueAsDouble());
    
  }
}

