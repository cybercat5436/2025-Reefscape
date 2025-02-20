// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase {
  /** Creates a new Algae. */
  private SparkMax ballMotor = new SparkMax (13, MotorType.kBrushless);
  private SparkMax armMotor = new SparkMax(14, MotorType.kBrushless); 
  SparkMaxConfig ballConfig;
  SparkMaxConfig armConfig;
  private SparkClosedLoopController armClosedLoopController;
  private RelativeEncoder encoder;
  private double targetPositionHigh = 56.81;
  private double targetPositionLow = 74.33;
  private double targetPositionProcessor = 1;
  private double targetVelocity = 500;
  private double stopVelocity = 0;

  

  public Algae() {
    ballConfig = new SparkMaxConfig();
    armConfig = new SparkMaxConfig();

    ballConfig
      .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);
    armConfig
      .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);
    ballMotor.configure(ballConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    encoder = armMotor.getEncoder(); 
    armClosedLoopController = armMotor.getClosedLoopController();
    armConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

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
    armConfig.closedLoop
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

    armConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(4000)
        .maxAcceleration(4000)
        .allowedClosedLoopError(1)
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
    armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Target PositionHigh", targetPositionHigh);
    SmartDashboard.setDefaultNumber("Target PositionLow", targetPositionLow);
    SmartDashboard.setDefaultNumber("Target PositionProcessor", targetPositionProcessor);
    SmartDashboard.setDefaultNumber("Target Velocity", targetVelocity);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);

  }
    public void algaeHigh() {
      armClosedLoopController.setReference(targetPositionHigh, ControlType.kMAXMotionPositionControl,
      ClosedLoopSlot.kSlot0);
      System.out.println("Algae High");
    }
    public void algaeHigh(double speed) {
      armMotor.set(speed);
    }
    public void algaeStop() {
      armMotor.set(0);
    }
    
    public void algaeLow() {
      armClosedLoopController.setReference(targetPositionLow, ControlType.kMAXMotionPositionControl,
      ClosedLoopSlot.kSlot0);
      System.out.println("Algae Low");
    }
    public void algaeProcessor() {
      armClosedLoopController.setReference(targetPositionProcessor, ControlType.kMAXMotionPositionControl,
      ClosedLoopSlot.kSlot0);
      System.out.println("Algae Processor");
    }
    // public void algaeStop() {
    //   armClosedLoopController.setReference(targetPositionLow, ControlType.kMAXMotionPositionControl,
    //   ClosedLoopSlot.kSlot0);
    //   armClosedLoopController.setReference(stopVelocity, ControlType.kMAXMotionVelocityControl,
    //   ClosedLoopSlot.kSlot1);
    //   System.out.println("Algae Stop");
    // }
    public void intakeBall(double speed) {
      ballMotor.set(speed);
    }
    public void releaseBall(double speed) {
      ballMotor.set(-speed);
    }
    public void stopBallMotor() {
      ballMotor.set(0);
    }
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Algae Encoder", (encoder.getPosition()));
    
  }
  
  

}
