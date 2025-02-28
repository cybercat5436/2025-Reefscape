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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
  final VoltageOut m_request = new VoltageOut(0);
  private double L1 = 0;
  private double L2 = 8.5;
  private double L3 = 17.6;
  private double L4 = 24;
  private int cycleCount;
 // private final TalonFXS elevator = new TalonFXS(12);
 private final SparkMax elevator=new SparkMax(57,MotorType.kBrushless);
 SparkMaxConfig motorConfig;
  private SparkClosedLoopController elevatorClosedLoopController;
  private RelativeEncoder encoder;


  public Elevator() {
     motorConfig = new SparkMaxConfig();
     encoder=elevator.getEncoder();


    motorConfig
      .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

    elevatorClosedLoopController = elevator.getClosedLoopController();
    motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);
    motorConfig.inverted(true);  

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(0.4)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    motorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(4000)
        .maxAcceleration(6000)
        .allowedClosedLoopError(0.5)
        // Set MAXMotion parameters for velocity control in slot 1
        .maxAcceleration(500, ClosedLoopSlot.kSlot1)
        .maxVelocity(6000, ClosedLoopSlot.kSlot1)
        .allowedClosedLoopError(1, ClosedLoopSlot.kSlot1);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    elevator.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

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
    elevatorClosedLoopController.setReference(L1, ControlType.kMAXMotionPositionControl,
    ClosedLoopSlot.kSlot0);
  } 
  public void raiseLevel2() {
    elevatorClosedLoopController.setReference(L2, ControlType.kMAXMotionPositionControl,
    ClosedLoopSlot.kSlot0);
  }
  public void raiseLevel3() {
    elevatorClosedLoopController.setReference(L3, ControlType.kMAXMotionPositionControl,
    ClosedLoopSlot.kSlot0);
  } 
  public void raiseLevel4() {
    elevatorClosedLoopController.setReference(L4, ControlType.kMAXMotionPositionControl,
    ClosedLoopSlot.kSlot0);
  } 
  public void stopElevator() {
    elevator.set(0);
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    super.periodic();
    SmartDashboard.putNumber("elevatorencoder", encoder.getPosition());
    SmartDashboard.putNumber("cycleCount", cycleCount++);
  }

 
  }

