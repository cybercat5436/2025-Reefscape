// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  private double L2 = 26.5;
  private double L3 = 65.64;
  private double L4 = 128;
  private final TalonFXS elevator = new TalonFXS(12);
  public Elevator() {
    var talonFXSConfigs = new TalonFXSConfiguration();
    talonFXSConfigs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    talonFXSConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    talonFXSConfigs.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Commutation;
    elevator.setPosition(0);
    // set slot 0 gains
    var slot0Configs = talonFXSConfigs.Slot0;
    slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
    slot0Configs.kP = 4.8;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;

    // set Motion Magic settings
    var motionMagicConfigs = talonFXSConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 300; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 1000; // 160 rps/s acceleration (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 200; // 1600 rps/s^2 jerk (0.1 seconds)

    elevator.getConfigurator().apply(talonFXSConfigs, 0.050);
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
    System.out.println("raised$$$$$$$$");
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
