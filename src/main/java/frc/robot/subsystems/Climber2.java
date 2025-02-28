// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber2 extends SubsystemBase {
  /** Creates a new Climber2. */
    private final TalonFX rightClimber = new TalonFX(15);
    private final TalonFX leftClimber = new TalonFX(16);
    private final DutyCycleOut leftOut = new DutyCycleOut(0);
    private final DutyCycleOut rightOut = new DutyCycleOut(0);
    private double rightEncoderValue;
    private double leftEncoderValue;
    private double ClimberEncoderLimitUp = 0;
    private double ClimberEncoderLimitDown = -5;

    /** Creates a new Climber. */
  public Climber2() {
    var leftConfiguration = new TalonFXConfiguration();
    var rightConfiguration = new TalonFXConfiguration();
  
    leftConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rightClimber.getConfigurator().apply(rightConfiguration);
    leftClimber.getConfigurator().apply(leftConfiguration);

    leftClimber.setPosition(0);
    rightClimber.setPosition(0);
  }
/**
 * This method makes the robot climb and score LOTS of points
  * 
  * 
  */
 public void leftClimb(double speed) {
  if((leftEncoderValue >= ClimberEncoderLimitUp) && (speed > 0)){
    speed = 0.0;
  }
  if((leftEncoderValue <= ClimberEncoderLimitDown) && (speed < 0)){
    speed = 0.0;
  }
  leftClimber.setControl(leftOut.withOutput(speed));
  SmartDashboard.putNumber("ClimberLeftSpeed", leftClimber.get());
 }


 public void rightClimb(double speed) {
  if((rightEncoderValue >= ClimberEncoderLimitUp) && (speed > 0)){
    speed = 0.0;
  }
  if((rightEncoderValue <= ClimberEncoderLimitDown) && (speed < 0)){
    speed = 0.0;
  }
  rightClimber.setControl(rightOut.withOutput(speed));
  SmartDashboard.putNumber("ClimberRightSpeed", rightClimber.get());
 }
 

public void stopClimb(){
leftOut.Output = 0;
rightOut.Output = 0;
leftClimber.setControl(leftOut);
rightClimber.setControl(rightOut);
System.out.println("Stopped climbing ");
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftEncoderValue = -leftClimber.getPosition().getValueAsDouble();
    rightEncoderValue = -rightClimber.getPosition().getValueAsDouble();

    SmartDashboard.putNumber("ClimberLeftValue",leftEncoderValue);
    SmartDashboard.putNumber("ClimberRightValue", rightEncoderValue);
  }
}
