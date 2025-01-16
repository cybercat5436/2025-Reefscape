// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber2;
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
    public final Climber climber = new Climber();
    public final Climber2 climber2 = new Climber2();

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
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
      
        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(joystick.getLeftY() * HalfSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(joystick.getLeftX() * HalfSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * HalfAngularRate) // Drive counterclockwise with negative X (left)
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
        // climber commands
        // joystick.y()
            // .whileTrue(new InstantCommand(() -> climber.climb(0.1)).repeatedly())
            // .onFalse(new InstantCommand(() -> climber.stopClimb()));
        // joystick.a()
        //     .whileTrue(new InstantCommand(() -> climber.climb(-0.7)).repeatedly())
        //     .onFalse(new InstantCommand(() -> climber.stopClimb()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // new Trigger(() -> (joystick.getRightY() > -0.2))
        joystick.y()
            .whileTrue(new InstantCommand(() -> climber2.rightClimb(-0.2)).repeatedly())
            .onFalse(new InstantCommand(() -> climber2.stopClimb()));
        

        // new Trigger(() -> (joystick.getRightY() < 0.2))
        joystick.x()
            .whileTrue(new InstantCommand(() -> climber2.rightClimb(0.2)).repeatedly())
            .onFalse(new InstantCommand(() -> climber2.stopClimb()));
            

        // new Trigger(() -> (joystick.getLeftY() > -0.2))
        joystick.b()
            .whileTrue(new InstantCommand(() -> climber2.leftClimb(-0.2)).repeatedly())
            .onFalse(new InstantCommand(() -> climber2.stopClimb()));
            

        // new Trigger(() -> (joystick.getLeftY() < 0.2))
        joystick.a()
            .whileTrue(new InstantCommand(() -> climber2.leftClimb(0.2)).repeatedly())
            .onFalse(new InstantCommand(() -> climber2.stopClimb()));
    }


    public Command getAutonomousCommand() {
        //return Commands.print("No autonomous command configured");
        System.out.println(autonChooser.getSelected().getName());
        return autonChooser.getSelected();
        
    }
}
