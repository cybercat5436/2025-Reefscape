// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdiSystem extends SubsystemBase {
  /** Creates a new CANdi. */
  private final CANBus kCANBus = new CANBus("rio");
  private final CANdi candi = new CANdi(23, kCANBus);
  public CANdiSystem() {
    var toApply = new CANdiConfiguration();

    toApply.DigitalInputs.S1FloatState = S1FloatStateValue.PullHigh; // Pulse-width sensor will drive low. Default of FloatDetect will typically work on most sensors.
    toApply.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenLow; // This example specifically assumes a hardware limit switch will close S2 to Ground. Default of CloseWhenNotFloating will also work

    toApply.PWM1.SensorDirection = true; // Invert the PWM1 position.
    toApply.PWM1.AbsoluteSensorDiscontinuityPoint = 0.75; // If the PWM 1 position on boot is after 0.75 rotations, treat it as x - 1 rotations.
                                                          // As an example, if the position is 0.87, it will boot to 0.87 - 1 = -0.13 rotations.

    /* User can change the configs if they want, or leave it empty for factory-default */
    candi.getConfigurator().apply(toApply);

    /* Speed up signals to an appropriate rate */
    BaseStatusSignal.setUpdateFrequencyForAll(100, candi.getPWM1Position(), candi.getPWM1Velocity(), candi.getS2State());


    /* Also configure TalonFX to use CANdi as a remote sensor for PWM 1, and Limit switch for S2 */
    var fxConfigs = new TalonFXConfiguration();
    fxConfigs.Feedback.withFusedCANdiPwm1(candi);
    fxConfigs.HardwareLimitSwitch.withReverseLimitRemoteCANdiS2(candi);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
