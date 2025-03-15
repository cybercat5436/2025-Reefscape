// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class AutoAlignCANdle extends SubsystemBase {
  /** Creates a new Sub. */
  private final CANdle m_candle = new CANdle(TunerConstants.kCANdleID, "rio");

  private static AutoAlignCANdle autoAlignCANdle = null;
  private boolean isAligned;
    private AutoAlignCANdle(){
      isAligned = false;
    }

    public static synchronized AutoAlignCANdle getInstance(){
        if (autoAlignCANdle == null){
            autoAlignCANdle = new AutoAlignCANdle();
        }
        return autoAlignCANdle;
    }
    
    public void setIsAligned(){
      isAligned = true;
    }
    public void showYellow() {
        m_candle.setLEDs(255,255,0);
    }
    public void showGreen(){
      m_candle.setLEDs(0,0,255);
    }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
