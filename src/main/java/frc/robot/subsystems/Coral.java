// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
  /** Creates a new Coral. */
  private SparkMax coralMotor = new SparkMax (37, MotorType.kBrushless);
  public Coral() {
    SparkMaxConfig coralConfig = new SparkMaxConfig();
    coralConfig
      .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);
    coralMotor.configure(coralConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void forward(double speed) {
    coralMotor.set(speed);

  }

  public void backward(double speed) {
    coralMotor.set(-speed);

  }

  public void stopMotor() {
    coralMotor.set(0);

  }
}
