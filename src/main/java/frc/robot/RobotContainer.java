// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.time.temporal.TemporalAccessor;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAlign;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber2;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.GamePieceDetector;
import frc.robot.subsystems.CANdleSystem.AnimationTypes;
import frc.robot.subsystems.CANdleSystem.AvailableColors;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.PoseUpdater;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.5; // kSpeedAt12Volts desired top speed
    private double HalfSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.25;
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double HalfAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) *0.5;

    private double robotX;
    private double robotY;
    private double kP = 0.03;
    private double ySpeed = 0;
    private double xSpeed = 0;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
   
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final LimeLight limeLightFront = new LimeLight("limelight-front");
    private final PoseUpdater poseUpdater = new PoseUpdater(limeLightFront, drivetrain);
    private final AutoAlign autoAlign = new AutoAlign(drivetrain,limeLightFront);
    private SendableChooser<Command> autonChooser;
    public final Climber climber = new Climber();
    public final Climber2 climber2 = new Climber2();
    public final GamePieceDetector coralSensor = new GamePieceDetector(4000, GamePieceDetector.Sensors.coral);
    //public final GamePieceDetector algaeSensor = new GamePieceDetector(3000, GamePieceDetector.Sensors.algae);
    public final Coral coral = new Coral();
    public final Algae algae = new Algae();
    public final Elevator elevator = new Elevator();
    public CANdleSystem candleSystem = new CANdleSystem(joystick.getHID());
    public RobotContainer(){
        autonChooser = AutoBuilder.buildAutoChooser("Test auton 2");
       SmartDashboard.putData("Auton Chooser", autonChooser);
    // autonChooser.addOption("Complex Auto", m_complexAuto);
    configureBindings();
    poseUpdater.enable();
    
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        joystick2.a()
          .whileTrue(new InstantCommand(() -> coral.forward(0.3)))
          .onFalse(new InstantCommand(() -> coral.stopMotor())); 

        joystick2.x()
           .whileTrue(new InstantCommand(() -> coral.backward(0.3)))
           .onFalse(new InstantCommand(() -> coral.stopMotor())); 

        joystick2.y()
           .whileTrue(new InstantCommand(() -> algae.intakeBall(0.3)))
           .onFalse(new InstantCommand(() -> algae.stopBallMotor()));

        joystick2.b()
           .whileTrue(new InstantCommand(() -> algae.releaseBall(0.3)))
           .onFalse(new InstantCommand(() -> algae.stopBallMotor()));
        joystick2.povUp()
            .onTrue(new InstantCommand(() -> elevator.raiseLevel1()))
            .onFalse(new InstantCommand(() -> elevator.stopElevator()));
        joystick2.povRight()
            .onTrue(new InstantCommand(() -> elevator.raiseLevel2()))
            .onFalse(new InstantCommand(() -> elevator.stopElevator()));

        joystick2.povDown()
            .onTrue(new InstantCommand(() -> elevator.raiseLevel3()))
            .onFalse(new InstantCommand(() -> elevator.stopElevator()));

        joystick2.povLeft()
            .onTrue(new InstantCommand(() -> elevator.raiseLevel4()))
            .onFalse(new InstantCommand(() -> elevator.stopElevator()));
        joystick2.rightBumper()
            .onTrue(new InstantCommand(() -> algae.raiseArmHigh()))
            .onFalse(new InstantCommand(() -> algae.stopArm()));
        joystick2.leftBumper()
            .onTrue(new InstantCommand(() -> algae.raiseArmLow()))
            .onFalse(new InstantCommand(() -> algae.stopArm()));
        joystick2.leftStick()
            .onTrue(new InstantCommand(() -> algae.armToProcesser()))
            .onFalse(new InstantCommand(() -> algae.stopArm()));
        SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(MaxSpeed);
        SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(MaxSpeed);
        SlewRateLimiter slewRateLimiterTurnX = new SlewRateLimiter(MaxSpeed);
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->{
                double xSpeed = slewRateLimiterX.calculate(joystick.getLeftY()* MaxSpeed);
                
                double ySpeed = slewRateLimiterY.calculate(joystick.getLeftX()* MaxSpeed);
                
                double yTurnSpeed = slewRateLimiterTurnX.calculate(joystick.getRightX()* MaxAngularRate);

                SmartDashboard.putNumber("xSpeed",xSpeed);
                SmartDashboard.putNumber("ySpeed",ySpeed);
                SmartDashboard.putNumber("yTurnSpeed",yTurnSpeed);

                return drive.withVelocityX(xSpeed * Math.abs(xSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(ySpeed * Math.abs(ySpeed)) // Drive left with negative X (left)
                    .withRotationalRate(-(yTurnSpeed * Math.abs(yTurnSpeed))); // Drive counterclockwise with negative X (left)
            }
            )
        );
    

        joystick.x().whileTrue(drivetrain.applyRequest(() -> {
        robotX = limeLightFront.getVisionArea();
        robotY = -limeLightFront.getVisionTargetHorizontalError();
        xSpeed = robotX * kP * MaxSpeed;
        ySpeed = robotY * kP * MaxSpeed;
            SmartDashboard.putNumber("robot y velocity", joystick.getLeftX() * MaxSpeed);
        return robotCentricDrive.withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(ySpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
        
    })
    );
//     joystick.back().whileTrue(drivetrain.applyRequest(() -> {
//         SmartDashboard.putNumber("robot y velocity", joystick.getLeftX() * MaxSpeed);
//     return robotCentricDrive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
//         .withVelocityY(joystick.getLefty) // Drive left with negative X (left)
//         .withRotationalRate(-joystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
        
// }));
        
      
        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() ->{
            
                double xSpeed = slewRateLimiterX.calculate(joystick.getLeftY()* HalfSpeed);
                
                
                double ySpeed = slewRateLimiterY.calculate(joystick.getLeftX()* HalfSpeed);
                
                
                double yTurnSpeed = slewRateLimiterTurnX.calculate(joystick.getRightX()* HalfAngularRate);
                SmartDashboard.putNumber("RBySpeed",ySpeed);
                SmartDashboard.putNumber("RBxSpeed",xSpeed);
                SmartDashboard.putNumber("RByTurnSpeed",yTurnSpeed);


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
        // climber commands
        // joystick.y()
            // .whileTrue(new InstantCommand(() -> climber.climb(0.1)).repeatedly())
            // .onFalse(new InstantCommand(() -> climber.stopClimb()));
        // joystick.a()
        //     .whileTrue(new InstantCommand(() -> climber.climb(-0.7)).repeatedly())
        //     .onFalse(new InstantCommand(() -> climber.stopClimb()));

        drivetrain.registerTelemetry(logger::telemeterize);
        SmartDashboard.putData("reset odymetry to 0,0",new InstantCommand(() -> drivetrain.resetPose(new Pose2d(0.0,0.0, new Rotation2d()))));
        

        // new Trigger(() -> (joystick.getRightY() > -0.2) new InstantCommand(() -> climber.rightClimb(0.2)));
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

        joystick.povLeft().onTrue(new InstantCommand(() -> candleSystem.changeAnimation(AnimationTypes.Fire)));
        joystick.povDown().onTrue(new InstantCommand(() -> candleSystem.turnOffColors()));
        joystick.povUp().onTrue(new InstantCommand(() -> candleSystem.showTeamColors()));
        /*joystick.povRight()
            .onTrue(new InstantCommand(() -> candleSystem.flashColor(AvailableColors.Red))
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(()->candleSystem.turnOffColors()))
            .andThen(new WaitCommand(0.25))
            .andThen(new InstantCommand(() -> candleSystem.flashColor(AvailableColors.Yellow)))
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(()->candleSystem.turnOffColors()))
            .andThen(new WaitCommand(0.25))
            .andThen(new InstantCommand(() -> candleSystem.flashColor(AvailableColors.Green)))
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(()->candleSystem.turnOffColors()))
            );*/

        Command blinkLight = Commands.repeatingSequence(
            new InstantCommand(() -> candleSystem.flashColor(AvailableColors.Red)),
            new WaitCommand(0.5),
            new InstantCommand(()->candleSystem.turnOffColors()),
            new WaitCommand(0.25)
        );


        //joystick.povRight().onTrue(new ParallelRaceGroup(blinkLight, new WaitCommand(2.25)));
        //joystick.povRight().onTrue(new ColorBlinkCommand(AvailableColors.Red, candleSystem));
    }


    public Command getAutonomousCommand() {
        //return Commands.print("No autonomous command configured");
        System.out.println(autonChooser.getSelected().getName());
        return autonChooser.getSelected();
        
    }

   
}

