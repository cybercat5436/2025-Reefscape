// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
public class AbsoluteEncoder extends SubsystemBase {
  /** Creates a new AbsoluteEncoder. */
  private static final double PRINT_PERIOD = 0.5; // Update every 500 ms
  private final CANcoder cancoder = new CANcoder(17, "rio");
  private double offset = 45; //Random value 
  public AbsoluteEncoder() {
      /* Configure CANcoder */
      var toApply = new CANcoderConfiguration();
      cancoder.setPosition(offset); //Zeroing the absolute encoder?   **Convert to degrees. 

      /* User can change the configs if they want, or leave it empty for factory-default */
      cancoder.getConfigurator().apply(toApply);
      
  
      /* Speed up signals to an appropriate rate */
      BaseStatusSignal.setUpdateFrequencyForAll(100, cancoder.getPosition(), cancoder.getVelocity());
   
  }

  public double getVelocityValue(){
    return cancoder.getVelocity(false).getValueAsDouble();
  }

  public StatusSignal<Angle> getPosition(){
    return cancoder.getPosition();
  }

  public StatusSignal<Angle> getAbsolutePosition(){
    return cancoder.getAbsolutePosition();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println("**************************************************");
    var pos = cancoder.getPosition();
    System.out.println("Position is " + pos.toString() + " with " + pos.getTimestamp().getLatency() + " seconds of latency");
    var vel = cancoder.getVelocity(false);
    System.out.println(
      "Velocity is " +
      vel.getValue() + " " +
      vel.getUnits() + " with " +
      vel.getTimestamp().getLatency() + " seconds of latency"
    );

    
  }


}
