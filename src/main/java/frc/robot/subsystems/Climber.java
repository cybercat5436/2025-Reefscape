// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final TalonFX leftLeader = new TalonFX(55);
    private final TalonFX rightFollower = new TalonFX(56);
    private final DutyCycleOut leftOut = new DutyCycleOut(0);
  /** Creates a new Climber. */
  public Climber() {
    var leftConfiguration = new TalonFXConfiguration();
    var rightConfiguration = new TalonFXConfiguration();
  
    leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
    leftLeader.getConfigurator().apply(leftConfiguration);
    rightFollower.getConfigurator().apply(rightConfiguration);
    rightFollower.setControl(new Follower(leftLeader.getDeviceID(), true));

    leftLeader.setSafetyEnabled(true);



  }
/**
 * This method makes the robot climb and score LOTS of points
  * 
  * 
  */
 public void climb(double speed) {
  leftOut.Output = speed;
  leftLeader.setControl(leftOut);
  SmartDashboard.putNumber("ClimberLeft", leftLeader.get());
  SmartDashboard.putNumber("ClimberLeftDuty", leftLeader.getDutyCycle().getValueAsDouble());
  SmartDashboard.putNumber("ClimberRight", rightFollower.get());
  System.out.println("climbing " + leftLeader.getDutyCycle().getValueAsDouble());
}

public void stopClimb(){
leftOut.Output = 0;
leftLeader.setControl(leftOut);
SmartDashboard.putNumber("ClimberLeft", leftLeader.get());
  SmartDashboard.putNumber("ClimberRight", rightFollower.get());
  System.out.println("Stopped ");
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
