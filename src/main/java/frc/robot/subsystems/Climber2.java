// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
    private double climberEncoderLimitUp = 10;
    private double climberEncoderLimitDown = -53;
    private double climberAutonStartPosition = -7.0;
    private MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);


    /** Creates a new Climber. */
  public Climber2() {
    var leftConfiguration = new TalonFXConfiguration();
    var rightConfiguration = new TalonFXConfiguration();
  
    leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leftConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

  

    MotionMagicConfigs mml = leftConfiguration.MotionMagic;
    mml.withMotionMagicCruiseVelocity(20) // 5 (mechanism) rotations per second cruise
      .withMotionMagicAcceleration(40) // Take approximately 0.5 seconds to reach max vel
      // Take approximately 0.1 seconds to reach max accel 
      .withMotionMagicJerk((200));

    Slot0Configs leftSlot0 = leftConfiguration.Slot0;
    leftSlot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    leftSlot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    leftSlot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    leftSlot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
    leftSlot0.kI = 0; // No output for integrated error
    leftSlot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

    MotionMagicConfigs mmr = rightConfiguration.MotionMagic;
    mmr.withMotionMagicCruiseVelocity(20) // 5 (mechanism) rotations per second cruise
      .withMotionMagicAcceleration(40) // Take approximately 0.5 seconds to reach max vel
      // Take approximately 0.1 seconds to reach max accel 
      .withMotionMagicJerk((200));

    Slot0Configs rightSlot0 = rightConfiguration.Slot0;
    rightSlot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    rightSlot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    rightSlot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    rightSlot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
    rightSlot0.kI = 0; // No output for integrated error
    rightSlot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output

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
  if((leftEncoderValue >= climberEncoderLimitUp) && (speed > 0)){
    speed = 0.0;
  }
  if((leftEncoderValue <= climberEncoderLimitDown) && (speed < 0)){
    speed = 0.0;
  }
  leftClimber.setControl(leftOut.withOutput(speed));
  SmartDashboard.putNumber("ClimberLeftSpeed", leftClimber.get());
 }


 public void rightClimb(double speed) {
  if((rightEncoderValue >= climberEncoderLimitUp) && (speed > 0)){
    speed = 0.0;
  }
  if((rightEncoderValue <= climberEncoderLimitDown) && (speed < 0)){
    speed = 0.0;
  }
  rightClimber.setControl(rightOut.withOutput(speed));
  SmartDashboard.putNumber("ClimberRightSpeed", rightClimber.get());
 }

 public void climberAutonStartPosition(){
  rightClimber.setControl(m_mmReq.withPosition(climberAutonStartPosition).withSlot(0));
  leftClimber.setControl(m_mmReq.withPosition(climberAutonStartPosition).withSlot(0));
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
    leftEncoderValue = leftClimber.getPosition().getValueAsDouble();
    rightEncoderValue = rightClimber.getPosition().getValueAsDouble();

    SmartDashboard.putNumber("ClimberLeftValue",leftEncoderValue);
    SmartDashboard.putNumber("ClimberRightValue", rightEncoderValue);
  }
}
