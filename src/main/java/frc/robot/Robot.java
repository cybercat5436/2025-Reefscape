// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private boolean didAutoChange = false;
  private boolean didAllianceChange = false;
  private Alliance m_alliance = Alliance.Blue;
  private Command generatePathCommand;

  public Robot() {
    m_robotContainer = new RobotContainer();
    this.generatePathCommand = m_robotContainer.testReefController();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    didAutoChange = m_autonomousCommand != m_robotContainer.getAutonomousCommand();
    didAllianceChange = m_alliance != m_robotContainer.getAlliance();
    if (didAutoChange || didAllianceChange){
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();
      m_alliance = m_robotContainer.getAlliance();
      System.out.println(String.format(
        "~~~ Updating Auto to %s Alliance: %s --> resetting odometry....",
        m_autonomousCommand.getName(), m_alliance.name()
      ));
      m_robotContainer.setOdometryPoseFromSelectedAuto(m_autonomousCommand.getName());
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    System.out.println("~~~~ Starting Auton: " + m_autonomousCommand.getName());

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    // m_robotContainer.resetGyroAfterAuton();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.resetSimulationStartPose();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    //m_robotContainer.resetSimulationStartPose();
    generatePathCommand.schedule();
  }
}
