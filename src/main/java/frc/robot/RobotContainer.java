// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RelativeTo;
import frc.robot.commands.PathingCommandGenerator;
import frc.robot.commands.PathingCommandGenerator.DifferentialOrientationMode;
import frc.robot.robotprofile.RobotProfile;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.Range;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;
import monologue.Logged;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Logged {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  PathingCommandGenerator path;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    path =
        new PathingCommandGenerator(
            new RobotProfile(3, 3, 2, 2, .9, .9),
            m_robotDrive::getPose,
            m_robotDrive::driveDifferential,
            DriveConstants.diffKinematics.trackWidthMeters,
            m_robotDrive,
            Field.CHARGED_UP_2023).setPhysicsAlgorithmType(false);
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY() * .5, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX() * .5, OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX() * .5, OIConstants.kDriveDeadband),
                    RelativeTo.kDriverRelative,
                    true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    SmartDashboard.putData("Reset Gyro", m_robotDrive.zeroHeadingCommand());
    // new JoystickButton(m_driverController, Button.kR1.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //         m_robotDrive));

    m_driverController.button(1).whileTrue(path.generateToPoseCommand(5.45, 3.15, Math.PI));
    m_driverController
        .button(2)
        .whileTrue(
            path.generateToDistFromPointCommand(
                new Translation2d(8, 4),
                new Range(.5, 1.5),
                new Rotation2d(Math.PI / 2),
                new Rotation2d(0),
                new Rotation2d(Math.PI / 2)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        path.generateToPoseCommand(6.3, 4.6, 0),
        new RunCommand(
                () -> m_robotDrive.drive(.25, 0, 0, RelativeTo.kRobotRelative, false), m_robotDrive)
            .withTimeout(.5),
        path.generateToPoseCommand(1.9, 4.5, Math.PI),
        path.generateToPoseCommand(6.3, 3.3, 0),
        new RunCommand(
                () -> m_robotDrive.drive(.25, 0, 0, RelativeTo.kRobotRelative, false), m_robotDrive)
            .withTimeout(.5),
        path.generateToPoseCommand(1.9, 3.3, Math.PI));
  }
}
