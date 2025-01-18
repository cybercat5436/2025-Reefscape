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

public class Climber2 extends SubsystemBase {
  /** Creates a new Climber2. */
    private final TalonFX rightClimber = new TalonFX(10);
    private final TalonFX leftClimber = new TalonFX(11);
    private final DutyCycleOut leftOut = new DutyCycleOut(0);
    private final DutyCycleOut rightOut = new DutyCycleOut(0);
  /** Creates a new Climber. */
  public Climber2() {
    var leftConfiguration = new TalonFXConfiguration();
    var rightConfiguration = new TalonFXConfiguration();
  
    leftConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    

  }
/**
 * This method makes the robot climb and score LOTS of points
  * 
  * 
  */
 public void leftClimb(double speed) {
  leftOut.Output = speed;
  leftClimber.setControl(leftOut);
  SmartDashboard.putNumber("ClimberLeft2", leftClimber.get());
 }
 public void rightClimb(double speed) {
  rightOut.Output = speed;
  rightClimber.setControl(rightOut);
  SmartDashboard.putNumber("ClimberRight2", rightClimber.get());
 }
 

public void stopClimb(){
leftOut.Output = 0;
rightOut.Output = 0;
leftClimber.setControl(leftOut);
rightClimber.setControl(rightOut);
SmartDashboard.putNumber("ClimberLeft", leftClimber.get());
  SmartDashboard.putNumber("ClimberRight", rightClimber.get());
  System.out.println("Stopped ");
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
