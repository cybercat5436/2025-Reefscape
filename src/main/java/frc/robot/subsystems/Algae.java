// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase {
  /** Creates a new Algae. */
  private SparkMax ballMotor = new SparkMax (39, MotorType.kBrushless);
  private final TalonFX armMotor = new TalonFX(38);
  MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
  final VoltageOut m_request = new VoltageOut(0);
  private double reefLow = 1;
  private double reefHigh = 2;
  private double processer = 3;
  

  public Algae() {
    SparkMaxConfig ballConfig = new SparkMaxConfig();
    ballConfig
      .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);
    ballMotor.configure(ballConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
  
    var talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
    slot0Configs.kP = 4.8;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 10; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 20; // 160 rps/s acceleration (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 200; // 1600 rps/s^2 jerk (0.1 seconds)

    armMotor.getConfigurator().apply(talonFXConfigs, 0.050);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeBall(double speed) {
    ballMotor.set(speed);

  }

  public void releaseBall(double speed) {
    ballMotor.set(-speed);

  }

  public void stopBallMotor() {
    ballMotor.set(0);

  }

  public void raiseArmLow() {
    m_motmag.Slot = 0;
    armMotor.setControl(m_motmag.withPosition(reefLow));
  } 
  
  public void raiseArmHigh() {
    m_motmag.Slot = 0;
    armMotor.setControl(m_motmag.withPosition(reefHigh));
  } 

  public void armToProcesser() {
    m_motmag.Slot = 0;
    armMotor.setControl(m_motmag.withPosition(processer));
  } 
  
  public void stopArm() {
    armMotor.setControl(m_request.withOutput(0));
    
  }
}
