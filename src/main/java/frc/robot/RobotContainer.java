// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.5; // kSpeedAt12Volts desired top speed
    private double HalfSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.25;
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double HalfAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) *0.5;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private SendableChooser<Command> autonChooser;
      
    public RobotContainer(){
        autonChooser = AutoBuilder.buildAutoChooser("Test auton 2");
       SmartDashboard.putData("Auton Chooser", autonChooser);
    // autonChooser.addOption("Complex Auto", m_complexAuto);

    configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->{
                SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(MaxSpeed);
                double xSpeed = slewRateLimiterX.calculate(joystick.getLeftY()* MaxSpeed);
                
                SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(MaxSpeed);
                double ySpeed = slewRateLimiterY.calculate(joystick.getLeftX()* MaxSpeed);
                
                SlewRateLimiter slewRateLimiterTurnX = new SlewRateLimiter(MaxSpeed);
                double yTurnSpeed = slewRateLimiterTurnX.calculate(joystick.getRightX()* MaxSpeed);

                return drive.withVelocityX(xSpeed * Math.abs(xSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(ySpeed * Math.abs(ySpeed)) // Drive left with negative X (left)
                    .withRotationalRate(-(yTurnSpeed * Math.abs(yTurnSpeed))); // Drive counterclockwise with negative X (left)
            }
            )
        );
    
        
      
        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() ->{
            SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(HalfSpeed);
                double xSpeed = slewRateLimiterX.calculate(joystick.getLeftY()* HalfSpeed);
                
                SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(HalfSpeed);
                double ySpeed = slewRateLimiterY.calculate(joystick.getLeftX()* HalfSpeed);
                
                SlewRateLimiter slewRateLimiterTurnX = new SlewRateLimiter(HalfSpeed);
                double yTurnSpeed = slewRateLimiterTurnX.calculate(joystick.getRightX()* HalfAngularRate);

        return drive.withVelocityX(xSpeed * Math.abs(xSpeed)) // Drive forward with negative Y (forward)
            .withVelocityY(ySpeed * Math.abs(ySpeed)) // Drive left with negative X (left)
            .withRotationalRate(-(yTurnSpeed * Math.abs(yTurnSpeed) ));
        } // Drive counterclockwise with negative X (left)
    ));
    
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        //return Commands.print("No autonomous command configured");
        System.out.println(autonChooser.getSelected().getName());
        return autonChooser.getSelected();
        
    }
}
