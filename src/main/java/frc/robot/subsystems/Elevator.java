// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
  final VoltageOut m_request = new VoltageOut(0);
  private double L1 = 0;
  private double L2 = 14.8;
  private double L3 = 34.4;
  private double L4 = 77.14;
  private final TalonFX elevator = new TalonFX(12);
  public Elevator() {
    // var talonFXConfigs = new TalonFXConfiguration();
    // talonFXConfigs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    // talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // talonFXConfigs.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Commutation;
    // elevator.setPosition(0);
    // // set slot 0 gains
    // var slot0Configs = talonFXConfigs.Slot0;
    // slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    // slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // // PID runs on position
    // slot0Configs.kP = 4.8;
    // slot0Configs.kI = 0;
    // slot0Configs.kD = 0.1;

    // set Motion Magic settings
    // var motionMagicConfigs = talonFXConfigs.MotionMagic;
    // motionMagicConfigs.MotionMagicCruiseVelocity = 300; // 80 rps cruise velocity
    // motionMagicConfigs.MotionMagicAcceleration = 1000; // 160 rps/s acceleration (0.5 seconds)
    // motionMagicConfigs.MotionMagicJerk = 200; // 1600 rps/s^2 jerk (0.1 seconds)

    // elevator.getConfigurator().apply(talonFXConfigs, 0.050);
     TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Configure gear ratio */
    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 12.8; // 12.8 rotor rotations per mechanism rotation

    /* Configure Motion Magic */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.withMotionMagicCruiseVelocity(60) // 5 (mechanism) rotations per second cruise
      .withMotionMagicAcceleration(120) // Take approximately 0.5 seconds to reach max vel
      // Take approximately 0.1 seconds to reach max accel 
      .withMotionMagicJerk(300);

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
    slot0.kI = 0; // No output for integrated error
    slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = elevator.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }
  // public void raiseLevel1() {
  //   m_motmag.Slot = 0;
  //   elevator.setControl(m_motmag.withPosition(L1));
  // } 
  
  public void raise() { 
    elevator.set(0.1);
  }
  public void lower() { 
    elevator.set(-0.1);
  }
  public void raiseLevel1() {
    m_motmag.Slot = 0;
    elevator.setControl(m_motmag.withPosition(L1));
  } 
  public void raiseLevel2() {
  m_motmag.Slot = 0;
   elevator.setControl(m_motmag.withPosition(L2));
  }
  public void raiseLevel3() {
    m_motmag.Slot = 0;
    elevator.setControl(m_motmag.withPosition(L3));
  } 
  public void raiseLevel4() {
    m_motmag.Slot = 0;
    elevator.setControl(m_motmag.withPosition(L4));
    System.out.println("$$$$$$$$$$$$$$raised level 4$$$$%$$$$$$$");
  } 
  public void stopElevator() {
    elevator.setControl(m_request.withOutput(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Rotor Encoder", elevator.getRotorPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Encoder", elevator.getPosition().getValueAsDouble());
    //SmartDashboard.putNumber("Elevator Quadrature Position", elevator.getRawQuadraturePosition().getValueAsDouble());
  }
}
