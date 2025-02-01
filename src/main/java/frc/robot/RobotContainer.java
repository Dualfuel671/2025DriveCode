// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RampSubSystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.WristSubSystem;
import frc.robot.subsystems.WristSubSystem.WristPosition;



public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 20% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);//TODO: verify port

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final WristSubSystem wristSubsystem = new WristSubSystem();
    private final RampSubSystem rampSubsystem = new RampSubSystem();

    private final Shooter shooter = new Shooter();
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverJoystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        driverJoystick.rightTrigger(.1).onTrue(new RunCommand(() -> elevatorSubsystem.setMotorSpeed(driverJoystick.getRightTriggerAxis())));
        driverJoystick.leftTrigger(.1).onTrue(new RunCommand(() -> elevatorSubsystem.setMotorSpeed(driverJoystick.getLeftTriggerAxis())));
        
        //Bind buttons for the ramp motor
        operatorJoystick.y().whileTrue(new RunCommand(() -> rampSubsystem.setRampSpeed(0.5)))
                            .whileFalse(new RunCommand(() -> rampSubsystem.setRampSpeed(0.0)));

        // Bind buttons for the wrist subsystem positions
        operatorJoystick.a().whileTrue(new RunCommand(() -> wristSubsystem.setWristPosition(WristPosition.LOW)));
        operatorJoystick.b().whileTrue(new RunCommand(() -> wristSubsystem.setWristPosition(WristPosition.MIDDLE)));
        operatorJoystick.x().whileTrue(new RunCommand(() -> wristSubsystem.setWristPosition(WristPosition.HIGH)));

        // Bind the Xbox controller triggers to control the elevator
         // I think I want the TriggerAxis value, not the boolean
        /*new Trigger(() -> joystick.getRightTriggerAxis() > 0.1)
            .whileTrue(new RunCommand(() -> elevatorSubsystem.setMotorSpeed(joystick.getRightTriggerAxis()), elevatorSubsystem));

        new Trigger(() -> joystick.getLeftTriggerAxis() > 0.1)
            .whileTrue(new RunCommand(() -> elevatorSubsystem.setMotorSpeed(-joystick.getLeftTriggerAxis()), elevatorSubsystem));
        */
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
