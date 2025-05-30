// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.lang.instrument.Instrumentation;
import java.time.Instant;

import org.json.simple.parser.ParseException;
import java.util.List;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoAlignWheelMovements;
import frc.robot.commands.AutoAlignWithLimelight;
import frc.robot.commands.CoralIntakeWithDetection;
import frc.robot.commands.DetectReefWithCANrange;
import frc.robot.commands.DriveForward;
import frc.robot.commands.FlashLEDsForAutoAlign;
import frc.robot.commands.StandardDeviation;
import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber2;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.GamePieceDetector;

import frc.robot.subsystems.CANdleSystem.AvailableColors;
import frc.robot.subsystems.ReefController.ReefPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.PoseUpdater;
import frc.robot.subsystems.ReefController;

public class RobotContainer {
    private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.35; // kSpeedAt12Volts desired top speed
    private double HalfSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.15;
    private double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.5; // 3/4 of a rotation per second max angular velocity
    private double HalfAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) *0.25;

    private double robotX;
    private double robotY;
    private double kP = 0.03;
    private double ySpeed = 0;
    private double xSpeed = 0;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withHeadingPID(3, 0, 0);
   
    private final Telemetry logger = new Telemetry(maxSpeed);
    // private final PhotonVision photonVision = new PhotonVision();
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    private final ReefController reefController = ReefController.getInstance();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    // private final LimeLight limeLightFront = new LimeLight("limelight-front", 0.02, -0.3, 0.65, -90.0, 0.0, 0.0);
    private final LimeLight limeLightFront = new LimeLight("limelight-front", 0.037, -0.236, 0.565, -91.5, -1, 2.5);
    // private final LimeLight limeLightFrontRight = new LimeLight("limelight-right", 0.162, 0.04, 0.33, -3.2, -16, 32.5);
    private final LimeLight limeLightFrontRight = new LimeLight("limelight-right", .127, 0.029, 0.395, 5.8, -21, 31.2);
    private final PoseUpdater poseUpdater = new PoseUpdater(limeLightFront, limeLightFrontRight, drivetrain);
    private final AutoAlign autoAlign = new AutoAlign(drivetrain,limeLightFront);
    private final DriveForward driveForward = new DriveForward(drivetrain, HalfSpeed, robotCentricDrive);
    private final AutoAlignWithLimelight autoALignWithLimelights = new AutoAlignWithLimelight(drivetrain,limeLightFront);
    private final StandardDeviation standardDeviation = new StandardDeviation(poseUpdater, drivetrain, new Pose2d(7.82,4.026,Rotation2d.k180deg),limeLightFront, limeLightFrontRight);
    private final FlashLEDsForAutoAlign flashLEDsForAutoAlign = new FlashLEDsForAutoAlign();
    
    private SendableChooser<Command> autonChooser;
    // public final Climber climber = new Climber();
    public final Climber2 climber2 = new Climber2();
    public final GamePieceDetector coralSensor = new GamePieceDetector(35000, GamePieceDetector.Sensors.coral);
    // public final GamePieceDetector algaeSensor = new GamePieceDetector(25000, GamePieceDetector.Sensors.algae, 0.1);
    public final GamePieceDetector reefDetector = new GamePieceDetector(1500, GamePieceDetector.Sensors.reef);
    
    public final Coral coral = new Coral();
    public final Algae algae = new Algae();
    public final Elevator elevator = new Elevator();
    private final DetectReefWithCANrange detectReefWithCANrange = new DetectReefWithCANrange(elevator, reefDetector);
    public CANdleSystem candleSystem = CANdleSystem.getInstance();

    private SequentialCommandGroup autoCoralHigh = new SequentialCommandGroup(
        new InstantCommand(() -> elevator.raiseLevel4())
        ,Commands.waitSeconds(2.5)
        ,Commands.print("Auto Coral High")
        .andThen(new InstantCommand(() -> coral.shoot(0.9)))
        , Commands.waitSeconds(0.5)
         .andThen(new InstantCommand(() -> coral.stopMotor())))
         .andThen(new InstantCommand(() -> elevator.stopElevator()));

    private SequentialCommandGroup detectReefL4Auton = 
        new InstantCommand(() -> elevator.raiseLevel4())
        .andThen(Commands.waitUntil(() -> elevator.atTargetHeight()).withTimeout(1))
        .andThen(Commands.waitSeconds(0.5))
        .andThen(new InstantCommand(() -> elevator.moveUpSlowly()))
        .andThen(Commands.waitUntil(() -> !reefDetector.isGamePieceClose).withTimeout(1))
        .andThen(Commands.waitSeconds(0.1))
        .andThen(new InstantCommand(() -> elevator.holdPosition()))
        // .andThen(new InstantCommand(() -> elevator.transferTargetHeight()))
        .andThen(new PrintCommand("This is running detectReefL4"))
        .andThen(new InstantCommand(() -> coral.shoot(1)))
        .andThen(Commands.waitSeconds(0.5)
        .andThen(new InstantCommand(() -> coral.stopMotor())));

    private SequentialCommandGroup reefL4 = 
        new InstantCommand(() -> elevator.raiseLevel4())
        .andThen(Commands.waitUntil(() -> elevator.atTargetHeight()).withTimeout(1))
        .andThen(Commands.waitSeconds(0.5))
        .andThen(new PrintCommand("This is running detectReefL4"))
        .andThen(new InstantCommand(() -> coral.shoot(1)))
        .andThen(Commands.waitSeconds(0.5)
        .andThen(new InstantCommand(() -> coral.stopMotor())));

    private SequentialCommandGroup detectReefL3 =
        new InstantCommand(() -> elevator.raiseLevel3())
        .andThen(Commands.waitUntil(() -> elevator.atTargetHeight()).withTimeout(1))
        .andThen(Commands.waitSeconds(0.5))
        .andThen(new InstantCommand(() -> elevator.moveUpSlowly()))
        .andThen(Commands.waitUntil(() -> !reefDetector.isGamePieceClose).withTimeout(1))
        // .andThen(Commands.waitSeconds(0.1))
        .andThen(new InstantCommand(() -> elevator.holdPosition()))
        // .andThen(new InstantCommand(() -> elevator.transferTargetHeight()))
        .andThen(new PrintCommand("This is running detectReefL3"))
        .andThen(new InstantCommand(() -> coral.shoot(0.50)))
        .andThen(Commands.waitSeconds(0.5)
        .andThen(new InstantCommand(() -> coral.stopMotor()))); // new InstantCommand(() -> elevator.raiseLevel3())
        // .andThen(new DetectReefWithCANrange(elevator, reefDetector))
        // .andThen(Commands.waitSeconds(0.5))
        // .andThen(new InstantCommand(() -> coral.backward(1)))
        // .andThen(Commands.waitSeconds(0.5)
        // .andThen(new InstantCommand(() -> coral.stopMotor()))));
        
    private SequentialCommandGroup detectReefL2 = 
        new InstantCommand(() -> elevator.raiseLevel2())
        .andThen(Commands.waitUntil(() -> elevator.atTargetHeight()).withTimeout(1))
        .andThen(Commands.waitSeconds(0.5))
        .andThen(new InstantCommand(() -> elevator.moveUpSlowly()))
        .andThen(Commands.waitUntil(() -> !reefDetector.isGamePieceClose).withTimeout(1))
        .andThen(Commands.waitSeconds(0.1))
        .andThen(new InstantCommand(() -> elevator.holdPosition()))
        // .andThen(new InstantCommand(() -> elevator.transferTargetHeight()))
        .andThen(new PrintCommand("This is running detectReefL2"))
        .andThen(new InstantCommand(() -> coral.shoot(0.50)))
        .andThen(Commands.waitSeconds(0.5)
        .andThen(new InstantCommand(() -> coral.stopMotor())));
   
       
               
           
    // private Command autoCoralHigh = Commands.sequence(
    //     new InstantCommand(() -> elevator.raiseLevel4())
    //     ,Commands.print("done raising.")
    //     ,Commands.waitSeconds(2.5)
    //     ,Commands.print("Done waiting")
    //     ,(new InstantCommand(() -> coral.backward(1)))
    //     ,Commands.waitSeconds(.5)
    //     ,(new InstantCommand(() -> coral.stopMotor()))
    //     ,(new InstantCommand(() -> elevator.stopElevator()))


    // );


    public RobotContainer(){
        // autonChooser.addOption("Complex Auto", m_complexAuto);
        configureBindings();
        poseUpdater.enable();
        registerNamedCommands();
        //testReefController();
        autonChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auton Chooser", autonChooser);

        drivetrain.registerTelemetry(logger::telemeterize);
        LimelightHelpers.setPipelineIndex(limeLightFront.limelightName, 1);
        LimelightHelpers.setPipelineIndex(limeLightFrontRight.limelightName, 1);

    }

        //Command autoCoralHigh = Commands.sequence(new InstantCommand(() -> elevator.raiseLevel4()), new InstantCommand(() -> coral.forward(1)));


    private void registerNamedCommands(){
        NamedCommands.registerCommand("printSomething", new InstantCommand(() -> System.out.println(">>>>>>>>>>>>>Printing Something")));
        NamedCommands.registerCommand("autoCoralHigh", autoCoralHigh);
        NamedCommands.registerCommand("autoAlign", autoAlign);
        NamedCommands.registerCommand("raiseArmHigh", new InstantCommand(() -> elevator.raiseLevel4()));
        NamedCommands.registerCommand("shootCoral", new InstantCommand(() -> coral.shoot(1)).andThen(Commands.waitSeconds(0.3)).andThen(new InstantCommand(() -> coral.stopMotor())));
        NamedCommands.registerCommand("lowerElevator", new InstantCommand(() -> elevator.raiseStartLevel()).andThen(Commands.waitSeconds(2).andThen(new InstantCommand(() -> elevator.stopElevator()))));
        NamedCommands.registerCommand("coralIntake", new CoralIntakeWithDetection(coral, coralSensor));
        NamedCommands.registerCommand("stopCoralIntake", new InstantCommand(() -> coral.intake(0)));
        NamedCommands.registerCommand("setClimberArmsToAutonStartPosition", 
        new InstantCommand(() -> climber2.climberAutonStartPosition())
        .andThen(Commands.waitSeconds(1.1))
        .andThen(new InstantCommand(() -> climber2.stopClimb())
        ));
        NamedCommands.registerCommand("autoAlignWithLimelight timeout 1.25", new AutoAlignWithLimelight(drivetrain, limeLightFront).withTimeout(1.25));
        NamedCommands.registerCommand("autoAlignWithLimelight timeout 2", new AutoAlignWithLimelight(drivetrain, limeLightFront).withTimeout(5));
        NamedCommands.registerCommand("driveForwardFor1Second", new  DriveForward(drivetrain, 2, robotCentricDrive));
        NamedCommands.registerCommand("faceWheels-120Degrees", new InstantCommand(() -> drivetrain.applyRequest(() ->
        point.withModuleDirection(new Rotation2d(-120)))));
        NamedCommands.registerCommand("reefDetection", detectReefL4Auton);
        

    }

    private void bindPrimaryController(){

        

        // *************************************************
        // Standard drive mode
        // *************************************************
        SlewRateLimiter slewRateLimiterX = new SlewRateLimiter(maxSpeed * 2);  //Note: setting slewratelimiter to 2x speed means it takes 0.5s to accelerate to full speed
        SlewRateLimiter slewRateLimiterY = new SlewRateLimiter(maxSpeed * 2);
        SlewRateLimiter slewRateLimiterTurnX = new SlewRateLimiter(maxAngularRate * 2);  //corrected from using MaxSpeed
        
        SlewRateLimiter slowModeSlewRateLimiterX = new SlewRateLimiter(maxSpeed * 5);  //Note: setting slewratelimiter to 2x speed means it takes 0.5s to accelerate to full speed
        SlewRateLimiter slowModeSlewRateLimiterY = new SlewRateLimiter(maxSpeed * 5);
        SlewRateLimiter slowModeSlewRateLimiterTurnX = new SlewRateLimiter(maxAngularRate * 3);  //corrected from using MaxSpeed
        
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->{
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                double xSpeed = slewRateLimiterX.calculate(-joystick.getLeftY()* maxSpeed);
                double ySpeed = slewRateLimiterY.calculate(-joystick.getLeftX()* maxSpeed);
                double yTurnSpeed = slewRateLimiterTurnX.calculate(joystick.getRightX()* maxAngularRate);

                // SmartDashboard.putNumber("xSpeed",xSpeed);
                // SmartDashboard.putNumber("ySpeed",ySpeed);
                // SmartDashboard.putNumber("yTurnSpeed",yTurnSpeed);

                return drive.withVelocityX(xSpeed * Math.abs(xSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(ySpeed * Math.abs(ySpeed)) // Drive left with negative X (left)
                    .withRotationalRate(-(yTurnSpeed * Math.abs(yTurnSpeed))); // Drive counterclockwise with negative X (left)
            })
        );

        // *************************************************
        // Fastmode driving
        // *************************************************
        // joystick.rightTrigger().whileTrue(drivetrain.applyRequest(() ->{
            
        //     double xSpeed = slowModeSlewRateLimiterX.calculate(-joystick.getLeftY()* HalfSpeed);
        //     double ySpeed = slowModeSlewRateLimiterY.calculate(-joystick.getLeftX()* HalfSpeed);
        //     double yTurnSpeed = slowModeSlewRateLimiterTurnX.calculate(joystick.getRightX()* HalfAngularRate);
            
        //     // SmartDashboard.putNumber("RBySpeed",ySpeed);
        //     // SmartDashboard.putNumber("RBxSpeed",xSpeed);
        //     // SmartDashboard.putNumber("RByTurnSpeed",yTurnSpeed);

        //     return drive.withVelocityX(xSpeed * Math.abs(xSpeed)) // Drive forward with negative Y (forward)
        //         .withVelocityY(ySpeed * Math.abs(ySpeed)) // Drive left with negative X (left)
        //         .withRotationalRate(-(yTurnSpeed * Math.abs(yTurnSpeed) ));
        // }));

        // *************************************************
        // SloMo driving
        // *************************************************
        joystick.leftTrigger().whileTrue(drivetrain.applyRequest(() ->{
            
            double xSpeed = slowModeSlewRateLimiterX.calculate(-joystick.getLeftY()* HalfSpeed);
            double ySpeed = slowModeSlewRateLimiterY.calculate(-joystick.getLeftX()* HalfSpeed);
            double yTurnSpeed = slowModeSlewRateLimiterTurnX.calculate(joystick.getRightX()* HalfAngularRate);
            
            // SmartDashboard.putNumber("RBySpeed",ySpeed);
            // SmartDashboard.putNumber("RBxSpeed",xSpeed);
            // SmartDashboard.putNumber("RByTurnSpeed",yTurnSpeed);

            return drive.withVelocityX(xSpeed * Math.abs(xSpeed)) // Drive forward with negative Y (forward)
                .withVelocityY(ySpeed * Math.abs(ySpeed)) // Drive left with negative X (left)
                .withRotationalRate(-(yTurnSpeed * Math.abs(yTurnSpeed) ));
        }));


        // *************************************************
        // AutoAlign to reef pole
        // *************************************************
        joystick.y().whileTrue(new AutoAlignWithLimelight(drivetrain, limeLightFront));
        
        
        // *************************************************
        // reset the field-centric heading on left bumper press
        // *************************************************
        joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        // *************************************************
        // Robot-Centric driving
        // *************************************************
        joystick.b().whileTrue(drivetrain.applyRequest(() -> {
            double xSpeed = slewRateLimiterX.calculate(-joystick.getLeftY()* maxSpeed);             
            double ySpeed = slewRateLimiterY.calculate(-joystick.getLeftX()* maxSpeed);          
            double yTurnSpeed = slewRateLimiterTurnX.calculate(joystick.getRightX()* maxAngularRate);

            return robotCentricDrive.withVelocityX(xSpeed * Math.abs(xSpeed)) 
            .withVelocityY(ySpeed * Math.abs(ySpeed))
            .withRotationalRate(-(yTurnSpeed * Math.abs(yTurnSpeed)));
        }));    


        // *************************************************
        // Heading-locked drive mode (for ReefController)
        // *************************************************
        // joystick.a().whileTrue(drivetrain.applyRequest(() -> {
        //     double xSpeed = slewRateLimiterX.calculate(-joystick.getLeftY()* maxSpeed);             
        //     double ySpeed = slewRateLimiterY.calculate(-joystick.getLeftX()* maxSpeed);          

        // return fieldCentricFacingAngle.withVelocityX(xSpeed * Math.abs(xSpeed)) 
        //     .withVelocityY(ySpeed * Math.abs(ySpeed)) 
        //     .withTargetDirection(reefController.getTargetRobotPose().getRotation());
        // }));
        

        // *************************************************
        // Brake mode
        // *************************************************
        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));


        // *************************************************
        // Point wheels but don't translate
        // *************************************************
        // joystick.x().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));
        

        // *************************************************
        // Reef Controller
        // *************************************************
        // joystick.povUp().onTrue(new InstantCommand(() -> reefController.setTargetReefPosition(ReefPosition.G))
        //         .andThen(getRumbleCommand()));
        // joystick.povDown().onTrue(new InstantCommand(()-> reefController.setTargetReefPosition(ReefPosition.A))
        //         .andThen(getRumbleCommand()));
        // joystick.povDownLeft().onTrue(new InstantCommand(()-> reefController.setTargetReefPosition(ReefPosition.L))
        //         .andThen(getRumbleCommand())
        //         .andThen(Commands.waitSeconds(0.1))
        //         .andThen(getRumbleCommand()));
        // joystick.povDownRight().onTrue(new InstantCommand(()-> reefController.setTargetReefPosition(ReefPosition.C))
        //         .andThen(getRumbleCommand())
        //         .andThen(Commands.waitSeconds(0.1))
        //         .andThen(getRumbleCommand()));
        // joystick.povUpLeft().onTrue(new InstantCommand(()-> reefController.setTargetReefPosition(ReefPosition.I))
        //         .andThen(getRumbleCommand())
        //         .andThen(Commands.waitSeconds(0.1))
        //         .andThen(getRumbleCommand()));
        // joystick.povUpRight().onTrue(new InstantCommand(()-> reefController.setTargetReefPosition(ReefPosition.E))
        //         .andThen(getRumbleCommand())
        //         .andThen(Commands.waitSeconds(0.1))
        //         .andThen(getRumbleCommand()));


        // *************************************************
        // LEDs
        // *************************************************
        joystick.povDown().onTrue(new InstantCommand(() -> candleSystem.decrementAnimation()));
        joystick.povUp().onTrue(new InstantCommand(() -> candleSystem.incrementAnimation()));

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

        //joystick.povRight().onTrue(new ParallelRaceGroup(getBlinkLightCommand(), new WaitCommand(2.25)));
        //joystick.povRight().onTrue(new ColorBlinkCommand(AvailableColors.Red, candleSystem));

    }

    private void bindSecondaryControls(){
        // *************************************************
        // Elevator Controls
        // *************************************************
        joystick2.x()
            .onTrue(new InstantCommand(() -> elevator.raiseLevel2())
            .andThen(new WaitUntilCommand(() -> elevator.atTargetHeight()))
            .andThen(new InstantCommand(() -> elevator.holdPosition()).withTimeout(0.25))
            .andThen(new InstantCommand(() -> elevator.stopElevator()))
            .andThen(new InstantCommand(() -> elevator.elevatorLevel = 0))
            .andThen(new InstantCommand(() -> {if(elevator.atTargetHeight()) elevator.resetEncoder();}))
            );
        
        // joystick2.a()
        //     .onTrue(new InstantCommand(() -> elevator.raiseLevel2()));
        // joystick2.b()
        //     .onTrue(new InstantCommand(() -> elevator.raiseLevel3()));
        joystick2.a()
            .onTrue(new InstantCommand(() -> elevator.raiseLevel2()) 
            );
        joystick2.b()
            .onTrue(new InstantCommand(() -> elevator.raiseLevel3()));
        joystick2.y()
            .onTrue(new InstantCommand(() -> elevator.raiseLevel4()));
        // joystick2.y().whileTrue(new InstantCommand(() -> elevator.moveUpSlowly()))
        // .onFalse(new InstantCommand(() -> elevator.stopElevator()));

        Trigger l4Trigger = new Trigger (() -> (joystick2.getLeftY() < -0.75));
        Trigger l3Trigger = new Trigger (() -> (joystick2.getLeftX() > 0.75));
        Trigger l2Trigger = new Trigger (() -> (joystick2.getLeftY() > 0.75));
        Trigger l1Trigger = new Trigger (() -> (joystick2.getLeftX() < -0.75));
        Trigger upAdjustmentTrigger = new Trigger (() -> (joystick2.getRightY() < -0.75));
        Trigger downAdjustmentTrigger = new Trigger (() -> (joystick2.getRightY() > 0.75));
       

        upAdjustmentTrigger.onTrue(new InstantCommand(() -> elevator.incrementHeightAdjustment()));
        downAdjustmentTrigger.onTrue(new InstantCommand(() -> elevator.decrementHeightAdjustment()));
        l4Trigger.onTrue(detectReefL4Auton);
        l3Trigger.onTrue(detectReefL3);
        l2Trigger.onTrue(detectReefL2);
        
        l1Trigger.onTrue(new InstantCommand(() -> elevator.raiseLevel2())
            .andThen(new WaitUntilCommand(() -> elevator.atTargetHeight()))
            .andThen(new InstantCommand(() -> elevator.holdPosition()).withTimeout(0.25))
            .andThen(new InstantCommand(() -> elevator.stopElevator()))
            .andThen(new InstantCommand(() -> elevator.elevatorLevel = 0))
            .andThen(new InstantCommand(() -> {if(elevator.atTargetHeight()) elevator.resetEncoder();}))
        );

        // *************************************************
        // Climber Controls
        // *************************************************
        // right/left for these triggers is ROBOT-CENTRIC
        // Trigger rightClimbUpTrigger = new Trigger (() -> (joystick2.getRightY() < -0.2));
        // Trigger rightClimbDownTrigger = new Trigger(() -> (joystick2.getRightY() > 0.2));
        // Trigger leftClimbUpTrigger = new Trigger(() -> (joystick2.getLeftY() < -0.2));
        // Trigger leftClimbDownTrigger = new Trigger(() -> (joystick2.getLeftY() > 0.2));
        // Trigger rightClimberStopTrigger = new Trigger(() -> joystick2.getRightY() >= -0.2 && joystick2.getRightY() <= 0.2);
        // Trigger leftClimberStopTrigger = new Trigger(() -> joystick2.getLeftY() >= -0.2 && joystick2.getLeftY() <= 0.2);

        // Motors controlled from these triggers are OPERATOR-CENTRIC
        // Because the robot is facing 180 during climb, the right/left sticks correspond to opposite side motors
        // leftClimbUpTrigger
        //     .whileTrue(new InstantCommand(() -> climber2.rightClimb(0.2)).repeatedly());
        // rightClimbUpTrigger
        //     .whileTrue(new InstantCommand(() -> climber2.leftClimb(0.2)).repeatedly());
        // leftClimbDownTrigger
        //     .whileTrue(new InstantCommand(() -> climber2.rightClimb(-0.2)).repeatedly());
        // rightClimbDownTrigger
        //     .whileTrue(new InstantCommand(() -> climber2.leftClimb(-0.2)).repeatedly());
        // rightClimberStopTrigger
        //     .whileTrue(new InstantCommand(() -> climber2.leftClimb(0)));
        // leftClimberStopTrigger
        //     .whileTrue(new InstantCommand(() -> climber2.rightClimb(0)));

        // Climber Encoder overrides
        // joystick2.back().and(leftClimbUpTrigger).whileTrue(new InstantCommand(() -> climber2.rightClimbOveride(0.2)).repeatedly());
        // joystick2.back().and(rightClimbUpTrigger).whileTrue(new InstantCommand(() -> climber2.leftClimbOveride(0.2)).repeatedly());
        // joystick2.back().and(leftClimbDownTrigger).whileTrue(new InstantCommand(() -> climber2.rightClimbOveride(-0.2)).repeatedly());
        // joystick2.back().and(rightClimbDownTrigger).whileTrue(new InstantCommand(() -> climber2.leftClimbOveride(-0.2)).repeatedly());
        // joystick2.back().and(leftClimberStopTrigger).whileTrue(new InstantCommand(() -> climber2.rightClimbOveride(0)));
        // joystick2.back().and(rightClimberStopTrigger).whileTrue(new InstantCommand(() -> climber2.leftClimbOveride(0)));
        
        joystick.povLeft().whileTrue(new InstantCommand(() -> climber2.rightClimb(0.2)).repeatedly())
        .onFalse(new InstantCommand(() -> climber2.stopClimb()));
        joystick.povRight().whileTrue(new InstantCommand(() -> climber2.rightClimb(-0.2)).repeatedly())
        .onFalse(new InstantCommand(() -> climber2.stopClimb()));
        joystick.b().whileTrue(new InstantCommand(() -> climber2.leftClimb(0.2)).repeatedly())
        .onFalse(new InstantCommand(() -> climber2.stopClimb()));
        joystick.x().whileTrue(new InstantCommand(() -> climber2.leftClimb(-0.2)).repeatedly())
        .onFalse(new InstantCommand(() -> climber2.stopClimb()));

        joystick.rightBumper().and(joystick.povRight()).whileTrue(new InstantCommand(() -> climber2.rightClimbOveride(-0.2)).repeatedly())
            .onFalse(new InstantCommand(() -> climber2.stopClimb()));
        joystick.rightBumper().and(joystick.x()).whileTrue(new InstantCommand(() -> climber2.leftClimbOveride(-0.2)).repeatedly())
            .onFalse(new InstantCommand(() -> climber2.stopClimb()));
        joystick.rightBumper().and(joystick.povLeft()).whileTrue(new InstantCommand(() -> climber2.rightClimbOveride(0.2)).repeatedly())
            .onFalse(new InstantCommand(() -> climber2.stopClimb()));
        joystick.rightBumper().and(joystick.b()).whileTrue(new InstantCommand(() -> climber2.leftClimbOveride(0.2)).repeatedly())
            .onFalse(new InstantCommand(() -> climber2.stopClimb()));
        
        Trigger climberArmsUp = new Trigger(() -> joystick2.getRightX() > 0.75);
        // climberArmsUp.whileTrue(new InstantCommand(() -> climber2.climberStartPosition(0.2)).repeatedly()
        // .until(climber2.climberStartPosition(0.2)))
        // .onFalse(new InstantCommand(() -> climber2.stopClimb()));

        climberArmsUp.whileTrue(Commands.waitUntil(() -> climber2.climberStartPosition(0.2))
            //.andThen(getRumbleCommand()
            //.andThen(getRumbleCommandJoystick2()))
            .andThen(new InstantCommand(() -> candleSystem.showGreen())));
        
        

        // *************************************************
        // Coral Controls
        // *************************************************
        // Trigger coralIntakeTrigger = new Trigger ((joystick2.leftTrigger()));

        joystick2.leftTrigger()
            .whileTrue(new InstantCommand(() -> {
                double speed = elevator.elevatorLevel == 4? 1 : 0.5;
                coral.shoot(speed);
            }))
            .onFalse(new InstantCommand(() -> coral.stopMotor()));

        joystick2.leftBumper()
                .whileTrue(new InstantCommand(() -> coral.intake(0.8)))
                .onFalse(new InstantCommand(() -> coral.stopMotor())); 
        
        Trigger coralIsPresent = new Trigger(() -> coralSensor.isGamePieceClose);
        coralIsPresent.onTrue(getRumbleCommand());

        // *************************************************
        // Algae Controls
        // *************************************************
        // Trigger algaeIntakeTrigger = new Trigger ((joystick2.rightTrigger()));

        joystick2.rightBumper()
            .whileTrue(new InstantCommand(() -> algae.intakeBall(1)))
            .onFalse(new InstantCommand(() -> algae.stopBallMotor()));

        joystick2.rightTrigger()
            .whileTrue(new InstantCommand(() -> algae.intakeBall(-0.7)))
            .onFalse(new InstantCommand(() -> algae.stopBallMotor()));

        joystick2.povUp()
            .onTrue(new InstantCommand(() -> algae.algaeHigh()))
            .onFalse(new InstantCommand(() -> algae.algaeStop()));
        joystick2.povDown()
            .onTrue(new InstantCommand(() -> algae.algaeLow()))
            .onFalse(new InstantCommand(() -> algae.algaeStop()));
        joystick2.povLeft()
            .onTrue(new InstantCommand(() -> algae.algaeStart()))
            .onFalse(new InstantCommand(() -> algae.algaeStop()));
        joystick2.povRight()
            .whileTrue(new InstantCommand(() -> algae.algaeProcessor()))
            .onFalse(new InstantCommand(() -> algae.algaeStop()));
        joystick2.povUpLeft()
            .whileTrue(new InstantCommand(() -> algae.algaeUp(0.3)))
            .onFalse(new InstantCommand(() -> algae.algaeStop()));  
        joystick2.povDownLeft()
            .whileTrue(new InstantCommand(() -> algae.algaeDown(0.3)))
            .onFalse(new InstantCommand(() -> algae.algaeStop()));
        

        // joystick2.povUp().and(joystick2.rightBumper())
        //     .whileTrue(new InstantCommand(() -> algae.releaseBall(-0.3))) 
        //     .onFalse(new InstantCommand(() -> algae.stopBallMotor()));
        // joystick2.povRight().and(joystick2.rightBumper())
        //     .whileTrue(new InstantCommand(() -> algae.releaseBall(-0.3))) 
        //     .onFalse(new InstantCommand(() -> algae.stopBallMotor()));
        
        //joystick2.povLeft()
         // .whileTrue(new InstantCommand(() -> algae.releaseBall(-0.3))) 
         //  .onFalse(new InstantCommand(() -> algae.stopBallMotor()));

        // joystick2.povUp()
        //     .whileTrue(new InstantCommand(() -> algae.algaeHigh()).repeatedly())
        //     .onFalse(new InstantCommand(() -> algae.algaeStop()));
        // joystick2.povLeft()
        // .onTrue(autoCoralHigh);

        // joystick2.povUp()
        //     .whileTrue(new InstantCommand(() -> algae.algaeHigh(-0.3)).repeatedly())
        //     .onFalse(new InstantCommand(() -> algae.algaeStop()));
        // joystick2.povDown()
        //     .whileTrue(new InstantCommand(() -> algae.algaeHigh(0.3)))
        //     .onFalse(new InstantCommand(() -> algae.algaeStop()));
    }

    private void bindCalibrationControls(){
        // only create controller if needed
        CommandXboxController calibrationJoystick = new CommandXboxController(4);

        // *************************************************
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // *************************************************
        calibrationJoystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        calibrationJoystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        calibrationJoystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        calibrationJoystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        Trigger l4Trigger = new Trigger (() -> (calibrationJoystick.getLeftY() < -0.75));
        Trigger l3Trigger = new Trigger (() -> (calibrationJoystick.getLeftX() > 0.75));
        Trigger l2Trigger = new Trigger (() -> (calibrationJoystick.getLeftY() > 0.75));
        Trigger l1Trigger = new Trigger (() -> (calibrationJoystick.getLeftX() < -0.75));
        Trigger upAdjustmentTrigger = new Trigger (() -> (calibrationJoystick.getRightY() < -0.75));
        Trigger downAdjustmentTrigger = new Trigger (() -> (calibrationJoystick.getRightY() > 0.75));

        l4Trigger.onTrue(detectReefL4Auton);
        l3Trigger.onTrue(detectReefL3);
        l2Trigger.onTrue(detectReefL2);
        
        l1Trigger.onTrue(new InstantCommand(() -> elevator.raiseLevel2())
            .andThen(new WaitUntilCommand(() -> elevator.atTargetHeight()))
            .andThen(new InstantCommand(() -> elevator.holdPosition()).withTimeout(0.25))
            .andThen(new InstantCommand(() -> elevator.stopElevator()))
            .andThen(new InstantCommand(() -> elevator.elevatorLevel = 0))
            .andThen(new InstantCommand(() -> {if(elevator.atTargetHeight()) elevator.resetEncoder();}))
        );

        calibrationJoystick.b().whileTrue(new InstantCommand(() -> coral.intake(0.8)))
        .onFalse(new InstantCommand(() -> coral.stopMotor()));
        


        // *************************************************
        // Camera calibration for apriltag pipeline
        // *************************************************
        // calibrationJoystick.a().whileTrue(standardDeviation);


        /*calibrationJoystick.a().whileTrue(
            new InstantCommand(() -> elevator.raiseLevel4())
            .andThen(Commands.waitUntil(() -> elevator.atTargetHeight()))
            .handleInterrupt(() -> System.out.println("~~~~~~~~~~   Interrupted!  ~~~~~~~~~~~~~~~"))
        );*/

        // calibrationJoystick.a().onTrue(new WheelMovementsTest(drivetrain, 0.3, robotCentricDrive, null));

        // *************************************************
        // Testing climber arm position for anti-coral position
        // *************************************************
        SmartDashboard.putData(" Climber arms to auton start position",
         new InstantCommand(() -> climber2.climberAutonStartPosition())
         .andThen(Commands.waitSeconds(1.1))
         .andThen(new InstantCommand(() -> climber2.stopClimb())
         ));
        SmartDashboard.putData(" detect reef L4", 
        new InstantCommand(() -> elevator.raiseLevel4())
        .andThen(Commands.waitUntil(() -> elevator.atTargetHeight()).withTimeout(1))
        .andThen(Commands.waitSeconds(0.5))
        .andThen(new InstantCommand(() -> elevator.moveUpSlowly()))
        .andThen(Commands.waitUntil(() -> !reefDetector.isGamePieceClose).withTimeout(1))
        .andThen(new InstantCommand(() -> elevator.holdPosition()))
        // .andThen(new InstantCommand(() -> elevator.transferTargetHeight()))
        .andThen(new PrintCommand("This is running detectReefL4"))
        .andThen(new InstantCommand(() -> coral.shoot(1)))
        .andThen(Commands.waitSeconds(0.5)
        .andThen(new InstantCommand(() -> coral.stopMotor()))));

         
        calibrationJoystick.x().onTrue(new AutoAlignWheelMovements(drivetrain, limeLightFront, reefController));
    }

    private void bindReefController(){
        Joystick reefPositionJoystick = new Joystick(2);
        Joystick reefLevelJoystick = new Joystick(3);

        Trigger button1 = new Trigger(() -> reefPositionJoystick.getRawButton(1));
        button1.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.A)));
        Trigger button2 = new Trigger(() -> reefPositionJoystick.getRawButton(2));
        button2.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.B)));
        Trigger button3 = new Trigger(() -> reefPositionJoystick.getRawButton(3));
        button3.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.C)));
        Trigger button4 = new Trigger(() -> reefPositionJoystick.getRawButton(4));
        button4.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.D)));
        Trigger button5 = new Trigger(() -> reefPositionJoystick.getRawButton(5));
        button5.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.E)));
        Trigger button6 = new Trigger(() -> reefPositionJoystick.getRawButton(6));
        button6.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.F)));
        Trigger button7 = new Trigger(() -> reefPositionJoystick.getRawButton(7));
        button7.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.G)));
        Trigger button8 = new Trigger(() -> reefPositionJoystick.getRawButton(8));
        button8.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.H)));
        Trigger button9 = new Trigger(() -> reefPositionJoystick.getRawButton(9));
        button9.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.I)));
        Trigger button10 = new Trigger(() -> reefPositionJoystick.getRawButton(10));
        button10.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.J)));
        Trigger button11 = new Trigger(() -> reefPositionJoystick.getRawButton(11));
        button11.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.K)));
        Trigger button12 = new Trigger(() -> reefPositionJoystick.getRawButton(12));
        button12.onTrue(new InstantCommand(() ->reefController.setTargetReefPosition(ReefPosition.L)));

       
        Trigger buttonLevel1Trigger = new Trigger(() -> reefLevelJoystick.getRawButton(1));
        buttonLevel1Trigger.onTrue(new InstantCommand(() ->reefController.setTargetReefLevel(1)));
        Trigger buttonLevel2Trigger = new Trigger(() -> reefLevelJoystick.getRawButton(2));
        buttonLevel2Trigger.onTrue(new InstantCommand(() ->reefController.setTargetReefLevel(2)));
        Trigger buttonLevel3Trigger = new Trigger(() -> reefLevelJoystick.getRawButton(3));
        buttonLevel3Trigger.onTrue(new InstantCommand(() ->reefController.setTargetReefLevel(3)));
        Trigger buttonLevel4Trigger = new Trigger(() -> reefLevelJoystick.getRawButton(4));
        buttonLevel4Trigger.onTrue(new InstantCommand(() ->reefController.setTargetReefLevel(4)));

    }

    private void configureBindings() {
        // Set the controller bindings

        bindPrimaryController();
        bindSecondaryControls();
        bindCalibrationControls();
        // bindReefController();
        
    }

    // private void testReefController(){
    //     reefController.setTargetReefPosition(ReefPosition.I);
    //     System.out.println(reefController.toString());

    //     reefController.setTargetReefPosition(ReefPosition.K);
    //     System.out.println(reefController.toString());

    //     reefController.setTargetReefPosition(ReefPosition.L);
    //     System.out.println(reefController.toString());

    //     reefController.setTargetReefPosition(ReefPosition.G);
    //     System.out.println(reefController.toString());
        
    //     reefController.setTargetReefPosition(ReefPosition.H);
    //     System.out.println(reefController.toString());        
    // }


    private Command getRumbleCommand(){
        return new InstantCommand(()-> joystick.setRumble(RumbleType.kBothRumble, 1))
        .andThen(Commands.waitSeconds(0.5))
        .andThen(new InstantCommand(()-> joystick.setRumble(RumbleType.kBothRumble, 0)));
    }
    private Command getRumbleCommandJoystick2(){
        return new InstantCommand(()-> joystick2.setRumble(RumbleType.kBothRumble, 1))
        .andThen(Commands.waitSeconds(0.5))
        .andThen(new InstantCommand(()-> joystick2.setRumble(RumbleType.kBothRumble, 0)));
    }
    private Command getBlinkLimeLightFrontCommand() {
        return new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceBlink(limeLightFront.limelightName))
        .andThen(Commands.waitSeconds(2))
        .andThen(new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOff(limeLightFront.limelightName)));
    };

    public void stopElevatorOnTeleInit() {
        elevator.stopElevator();
    }
       
    
    private Command getBlinkLightCommand(){
        return Commands.repeatingSequence(
            new InstantCommand(() -> candleSystem.flashColor(AvailableColors.Green)),
            new WaitCommand(0.5),
            new InstantCommand(()->candleSystem.turnOffColors()),
            new WaitCommand(0.25)
        );
    }


    public void setOdometryPoseFromSelectedAuto(String autonName){
        System.out.println("~~~~Setting pose from Selected Auto: " + autonName);
        Pose2d startPose = new Pose2d();
        
        try {
    
          List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autonName);
          startPose = getFirstPose(paths);
    
        } catch (IOException e){
          // Exception thrown if starting pose is null in PathPlanner Auton file
          System.out.println("IOException No starting pose detected in Auton file!");
        } catch (ParseException e){
            // Exception thrown if starting pose is null in PathPlanner Auton file
            System.out.println("ParseException No starting pose detected in Auton file!");
        }
        System.out.println("~~~~ New starting pose: " + startPose.toString());
        drivetrain.resetPose(startPose);
    }


    public Alliance getAlliance(){
        if(!DriverStation.isDSAttached()){
            // System.out.println("~~~~~" + DriverStationSim.getAllianceStationId());
            // *** NOTE: if using simGUI set alliance manually here ****
            // System.out.println(String.format("isRedAlliance: %s", 
            // NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(false)
            // ));
            boolean isRed = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(false);
            return isRed? Alliance.Red : Alliance.Blue;
        }
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }


    private Pose2d getFirstPose(List<PathPlannerPath> paths){
        Pose2d firstPose = new Pose2d();
        // Guard against empty paths list
        if(paths.isEmpty()){
            System.out.println("Auto has no paths, returing origin");
            return new Pose2d();
        }

        System.out.println("~~~~ Is Driver Station Attached: " + DriverStation.isDSAttached());
        // figure out if path flipping has to happen
        boolean shouldFlip = (getAlliance() == DriverStation.Alliance.Red);
        System.out.println("~~~~~ Should Flip Path: " + shouldFlip);

        PathPlannerPath firstPath = shouldFlip ? paths.get(0).flipPath() : paths.get(0);
        // get the ideal starting state
        var firstRotation = firstPath.getIdealStartingState().rotation();
        // use position from first pose and add ideal starting state
        firstPose = new Pose2d(firstPath.getAllPathPoints().get(0).position, firstRotation);

        return firstPose;
    }
        

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
        
    }
}

