// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism.DisabledReason;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.GamePieceDetector.Sensors;

public class CoralPoleDetector extends SubsystemBase {
  /** Creates a new CoralPoleDetector. */
  private double distanceThreshold = 0.2;
  private Sensors sensor;
  private CANrange detector = new CANrange(49);
  private double distanceToPole;
  
  public CoralPoleDetector() {

    CANrangeConfiguration config = new CANrangeConfiguration();
    
    this.sensor = sensor;
   
    
    
   
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    distanceToPole = detector.getDistance().getValueAsDouble();
    System.out.println(distanceToPole);
  }
}
