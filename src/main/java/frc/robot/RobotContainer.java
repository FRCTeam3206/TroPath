// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RelativeTo;
import frc.robot.commands.PathingCommand;
import frc.robot.robotprofile.Motor;
import frc.robot.robotprofile.RobotProfile;
import frc.robot.subsystems.DriveSubsystem;
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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    PathingCommand.setDefaultRobotProfile(
        new RobotProfile(50, 3 / 39.37, .9, .9, Motor.NEO().gear(Motor.REV_HIGH))
            .setSafteyMultiplier(.8));
    System.out.println(PathingCommand.getDefaultRobotProfile());
    PathingCommand.setRobot(() -> m_robotDrive.getPose(), m_robotDrive::driveSpeed, m_robotDrive);
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

    m_driverController
        .button(1)
        .whileTrue(new PathingCommand(2.3, 4.5, Math.PI).setContinnuous(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        new PathingCommand(new Pose2d(6.3, 4.6, new Rotation2d())),
        new RunCommand(
                () -> m_robotDrive.drive(.25, 0, 0, RelativeTo.kRobotRelative, false), m_robotDrive)
            .withTimeout(.5),
        new PathingCommand(new Pose2d(1.9, 4.5, new Rotation2d(Math.PI))),
        new PathingCommand(new Pose2d(6.3, 3.3, new Rotation2d())),
        new RunCommand(
                () -> m_robotDrive.drive(.25, 0, 0, RelativeTo.kRobotRelative, false), m_robotDrive)
            .withTimeout(.5),
        new PathingCommand(new Pose2d(1.9, 3.3, new Rotation2d(Math.PI))));
  }
}
