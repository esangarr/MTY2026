// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.LimeCommands;
import frc.robot.DriveTrain.SwerveConstants.TunerConstants;
import frc.robot.DriveTrain.SwerveSubs.CommandSwerveDrivetrain;
import frc.robot.DriveTrain.SwerveSubs.PoseFinder;
import frc.robot.DriveTrain.SwerveSubs.SwerveRequestFactory;

import frc.robot.DriveTrain.SwerveSubs.PoseFinder;

public class RobotContainer {
 
    private final double MaxSpeed = SwerveRequestFactory.MaxSpeed;
    private final double MaxAngularRate = SwerveRequestFactory.MaxAngularRate;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();        
    }

    private void configureBindings() {

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true) );
        // -------- SYSID CARACTESIZATION CONTROL -----------------------------------------------------------------------
        driver.a().whileTrue(drivetrain.applyRequest(() -> SwerveRequestFactory.brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->SwerveRequestFactory.point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                SwerveRequestFactory.driveFieldCentric.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
        driver.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        driver.rightBumper().whileTrue(LimeCommands.snapToApril(drivetrain));

        driver.x().whileTrue(drivetrain.getPoseFinder().toPose(new Pose2d(2.159, 4.650, Rotation2d.kZero)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                SwerveRequestFactory.driveFieldCentric.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
