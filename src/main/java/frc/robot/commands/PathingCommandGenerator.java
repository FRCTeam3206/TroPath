package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.robotprofile.RobotProfile;
import frc.utils.AllianceUtil;
import java.util.function.Consumer;
import java.util.function.Supplier;

import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;

/**
 * Configured with settings and references in order to be able to generate multiple PathingCommands
 * to different goals.
 */
public class PathingCommandGenerator {
  private final RobotProfile robotProfile;
  private final Supplier<Pose2d> robotPose;
  private Consumer<ChassisSpeeds> drive;
  private PathfinderBuilder builder;
  private final Subsystem subsystem;
  private double translationTolerance = .05, rotationTolerance = Math.PI / 32;
  private boolean allianceFlip = true;
  private boolean isDifferential = false;
  /**
   * Constructs a PathingCommandGenerator to generate {@code PathingCommand}s on a holonomic chassis with the given
   * settings. Defaults to using the layout for the 2024 field. To use a custom field, add String or
   * Field as a last parameter in the constructor.
   *
   * @param robotProfile The {@link RobotProfile} to be used by this command generator.
   * @param robotPose Supplier of the robot's current position as a {@link Pose2d}. For example, a
   *     reference to a {@code getPose()} method.
   * @param drive Consumer to drive the robot. Must take {@link ChassisSpeeds} and be field
   *     relative. For example, a reference to a {@code drive()} method.
   * @param subsystem The drive subsystem (so it can be required).
   */
  public PathingCommandGenerator(
      RobotProfile robotProfile,
      Supplier<Pose2d> robotPose,
      Consumer<ChassisSpeeds> drive,
      Subsystem subsystem) {
    this(robotProfile, robotPose, drive, subsystem, Field.CRESCENDO_2024);
  }

  /**
   * Constructs a PathingCommandGenerator to generate {@code PathingCommand}s on a holonomic chassis with the given
   * settings. Creates a custom field from the given json name. To use the default field, leave out
   * the String from the constructor. To use a custom field from the Field enum, replace the String
   * value with a field value.
   *
   * @param robotProfile The {@link RobotProfile} to be used by this command generator.
   * @param robotPose Supplier of the robot's current position as a {@link Pose2d}. For example, a
   *     reference to a {@code getPose()} method.
   * @param drive Consumer to drive the robot. Must take {@link ChassisSpeeds} and be field
   *     relative. For example, a reference to a {@code drive()} method.
   * @param subsystem The drive subsystem (so it can be required).
   * @param fieldJsonName The name the custom field json file, which must be located in the deploy
   *     folder. NOT the full path. For example, {@code "my_field.json"}.
   */
  public PathingCommandGenerator(
      RobotProfile robotProfile,
      Supplier<Pose2d> robotPose,
      Consumer<ChassisSpeeds> drive,
      Subsystem subsystem,
      String fieldJsonName) {
    this(
        robotProfile,
        robotPose,
        drive,
        subsystem,
        new PathfinderBuilder(Filesystem.getDeployDirectory() + "\\" + fieldJsonName));
  }

  /**
   * Constructs a PathingCommandGenerator to generate {@code PathingCommand}s on a holonomic chassis with the given
   * settings. Creates a custom field from the given json name. To use the default field, leave out
   * the String from the constructor. To use a custom field from the Field enum, replace the String
   * value with a field value.
   *
   * @param robotProfile The {@link RobotProfile} to be used by this command generator.
   * @param robotPose Supplier of the robot's current position as a {@link Pose2d}. For example, a
   *     reference to a {@code getPose()} method.
   * @param drive Consumer to drive the robot. Must take {@link ChassisSpeeds} and be field
   *     relative. For example, a reference to a {@code drive()} method.
   * @param subsystem The drive subsystem (so it can be required).
   * @param field The value of the desired field from the {@link Field} enum.
   */
  public PathingCommandGenerator(
      RobotProfile robotProfile,
      Supplier<Pose2d> robotPose,
      Consumer<ChassisSpeeds> drive,
      Subsystem subsystem,
      Field field) {
    this(robotProfile, robotPose, drive, subsystem, new PathfinderBuilder(field));
  }

  /**
   * Please only use this constructor if you understand well how the code works and know why you
   * don't want to use one of the other constructors. Constructs a PathingCommandGenerator to
   * generate {@code PathingCommand}s on a holonomic chassis with the given settings. This constructor allows you to pass
   * in a custom Pathfinder builder instead of one being created with a field layout as the default,
   * with a custom json, or with a Field value. The builder can later be modified using {@link
   * #getBuilder}).
   *
   * @param robotProfile The {@link RobotProfile} to be used by this command generator.
   * @param robotPose Supplier of the robot's current position as a {@link Pose2d}. For example, a
   *     reference to a {@code getPose()} method.
   * @param drive Consumer to drive the robot. Must take {@link ChassisSpeeds} and be field
   *     relative. For example, a reference to a {@code drive()} method.
   * @param subsystem The drive subsystem (so it can be required).
   * @param builder The PathfinderBuilder to be used by this command generator.
   */
  public PathingCommandGenerator(
      RobotProfile robotProfile,
      Supplier<Pose2d> robotPose,
      Consumer<ChassisSpeeds> drive,
      Subsystem subsystem,
      PathfinderBuilder builder) {
    this.robotProfile = robotProfile;
    this.robotPose = robotPose;
    this.drive = drive;
    this.subsystem = subsystem;
    AllianceUtil.setRobot(robotPose);
    this.builder = builder;
  }
  /**
   * Constructs a PathingCommandGenerator to generate {@code PathingCommand}s on a differential chassis with the given
   * settings. Defaults to using the layout for the 2024 field. To use a custom field, add String or
   * Field as a last parameter in the constructor.
   *
   * @param robotProfile The {@link RobotProfile} to be used by this command generator.
   * @param robotPose Supplier of the robot's current position as a {@link Pose2d}. For example, a
   *     reference to a {@code getPose()} method.
   * @param drive Consumer to drive the robot. Must take {@link DifferentialDriveWheelSpeeds} in meters per second
   * @param trackWidth The wheel to wheel distance on the drivetrain, in meters
   * @param subsystem The drive subsystem (so it can be required).
   */
  public PathingCommandGenerator(
      RobotProfile robotProfile,
      Supplier<Pose2d> robotPose,
      Consumer<DifferentialDriveWheelSpeeds> drive,
      double trackWidth,
      Subsystem subsystem) {
    this(robotProfile, robotPose, drive, trackWidth, subsystem, Field.CRESCENDO_2024);
  }
  /**
   * Constructs a PathingCommandGenerator to generate {@code PathingCommand}s on a differential chassis with the given
   * settings. Creates a custom field from the given json name. To use the default field, leave out
   * the String from the constructor. To use a custom field from the Field enum, replace the String
   * value with a field value.
   *
   * @param robotProfile The {@link RobotProfile} to be used by this command generator.
   * @param robotPose Supplier of the robot's current position as a {@link Pose2d}. For example, a
   *     reference to a {@code getPose()} method.
   * @param drive Consumer to drive the robot. Must take {@link DifferentialDriveWheelSpeeds} in meters per second
   * @param trackWidth The wheel to wheel distance on the drivetrain, in meters
   * @param subsystem The drive subsystem (so it can be required).
   * @param fieldJsonName The name the custom field json file, which must be located in the deploy
   *     folder. NOT the full path. For example, {@code "my_field.json"}.
   */
  public PathingCommandGenerator(
      RobotProfile robotProfile,
      Supplier<Pose2d> robotPose,
      Consumer<DifferentialDriveWheelSpeeds> drive,
      double trackWidth,
      Subsystem subsystem,
      String fieldJsonName) {
    this(
        robotProfile,
        robotPose,
        drive,
        trackWidth,
        subsystem,
        new PathfinderBuilder(Filesystem.getDeployDirectory() + "\\" + fieldJsonName));
  }
  /**
   * Constructs a PathingCommandGenerator to generate {@code PathingCommand}s on a differential chassis with the given
   * settings. Creates a custom field from the given json name. To use the default field, leave out
   * the String from the constructor. To use a custom field from the Field enum, replace the String
   * value with a field value.
   *
   * @param robotProfile The {@link RobotProfile} to be used by this command generator.
   * @param robotPose Supplier of the robot's current position as a {@link Pose2d}. For example, a
   *     reference to a {@code getPose()} method.
   * @param drive Consumer to drive the robot. Must take {@link DifferentialDriveWheelSpeeds} in meters per second
   * @param trackWidth The wheel to wheel distance on the drivetrain, in meters
   * @param subsystem The drive subsystem (so it can be required).
   * @param field The value of the desired field from the {@link Field} enum.
   */
  public PathingCommandGenerator(
      RobotProfile robotProfile,
      Supplier<Pose2d> robotPose,
      Consumer<DifferentialDriveWheelSpeeds> drive,
      double trackWidth,
      Subsystem subsystem,
      Field field) {
    this(robotProfile, robotPose, drive,trackWidth, subsystem, new PathfinderBuilder(field));
  }
  /**
   * Only use this if you know what you are doing. Constructs a PathingCommandGenerator to
   * generate {@code PathingCommand}s on a differential chassis with the given settings. This constructor allows you to pass
   * in a custom Pathfinder builder instead of one being created with a field layout as the default,
   * with a custom json, or with a Field value. The builder can later be modified using {@link
   * #getBuilder}).
   *
   * @param robotProfile The {@link RobotProfile} to be used by this command generator.
   * @param robotPose Supplier of the robot's current position as a {@link Pose2d}. For example, a
   *     reference to a {@code getPose()} method.
   * @param drive Consumer to drive the robot. Must take {@link DifferentialDriveWheelSpeeds} in meters per second
   * @param trackWidth The wheel to wheel distance on the drivetrain, in meters
   * @param subsystem The drive subsystem (so it can be required).
   * @param builder The PathfinderBuilder to be used by this command generator.
   */
  public PathingCommandGenerator(
      RobotProfile robotProfile,
      Supplier<Pose2d> robotPose,
      Consumer<DifferentialDriveWheelSpeeds> drive,
      double trackWidth,
      Subsystem subsystem,
      PathfinderBuilder builder) {
    this.robotProfile = robotProfile;
    this.robotPose = robotPose;
    setDifferentialDrive(drive, trackWidth);
    this.subsystem = subsystem;
    AllianceUtil.setRobot(robotPose);
    this.builder = builder;
  }
  private Consumer<ChassisSpeeds> differentialRotationConsumer;
  public void setDifferentialDrive(Consumer<DifferentialDriveWheelSpeeds> diffDrive,double trackWidth){
    isDifferential=true;
    DifferentialDriveKinematics kinematics=new DifferentialDriveKinematics(trackWidth);
    drive=(ChassisSpeeds speeds)->{
      double theta=Math.atan2(speeds.vyMetersPerSecond,speeds.vxMetersPerSecond)-robotPose.get().getRotation().getRadians();
      double velocity=Math.sqrt(speeds.vxMetersPerSecond*speeds.vxMetersPerSecond+speeds.vyMetersPerSecond*speeds.vyMetersPerSecond);
      double linearVelocity=velocity*Math.cos(theta);
      double rotationalVelocity=(velocity)*Math.sin(theta)*2/trackWidth;
      DifferentialDriveWheelSpeeds output=kinematics.toWheelSpeeds(new ChassisSpeeds(linearVelocity, 0, rotationalVelocity));
      diffDrive.accept(output);
    };
    differentialRotationConsumer=(ChassisSpeeds speeds)->{
      diffDrive.accept(kinematics.toWheelSpeeds(new ChassisSpeeds(0, 0, speeds.omegaRadiansPerSecond)));
    };
  }
  /**
   * Sets the tolerances for this PathingCommandGenerator. These default to 5 cm and pi / 32
   * radians. The tolerances are the maximum allowed error for which the robot is considered to have
   * reached the goal and should be tuned to your robot. They should be as small as possible without
   * being more precise than the robot can achieve well. If the tolerance is too small, the robot
   * will spend longer than it should trying to get perfectly in position (and move the wheels in
   * different directions as it tries to perfectly adjust). If it is at a good amount, it should
   * stop as soon as it reaches the position (the wheels moving back and forth should not be
   * noticeable).
   *
   * @param translationTolerance The translation tolerance to set. In meters. Defaults to 5 cm.
   * @param rotationTolerance The rotation tolerance to set. In radians. Defaults to pi/32.
   * @return The generator.
   */
  public PathingCommandGenerator setTolerances(
      double translationTolerance, double rotationTolerance) {
    this.translationTolerance = translationTolerance;
    this.rotationTolerance = rotationTolerance;
    return this;
  }

  /**
   * Set whether to enable flipping the position based on the alliance. This defaults to true and
   *
   * @param flippingEnabled Whether to enable alliance flipping.
   * @return The generator.
   */
  public PathingCommandGenerator setAllianceFlipping(boolean flippingEnabled) {
    allianceFlip = flippingEnabled;
    return this;
  }

  private Pose2d getPoseForAlliance(Pose2d pose) {
    if (allianceFlip) return AllianceUtil.getPoseForAlliance(pose);
    return pose;
  }

  /**
   * Allows you to modify the builder using its methods. For example, you could do {@code
   * generator.getBuilder().setPointSpacing(0.2)}.
   *
   * @return The builder used in this PathingCommandGenerator.
   */
  public PathfinderBuilder getBuilder() {
    return builder;
  }

  /**
   * Makes a new PathingCommandGenerator with the same settings. Please note that the builders are
   * still linked, so if you change this one's builder, the builder of the one it was created from
   * will be changed too.
   *
   * @param givenTranslationTolerance The translational tolerance for the new
   *     PathingCommandGenerator.
   * @param givenRotationTolerance The rotational tolerance for the new PathingCommandGenerator.
   * @return A new PathingCommandGenerator with different tolerances.
   */
  public PathingCommandGenerator withModifiedTolerances(
      double givenTranslationTolerance, double givenRotationTolerance) {
    return new PathingCommandGenerator(robotProfile, robotPose, drive, subsystem, builder)
        .setTolerances(givenRotationTolerance, givenRotationTolerance)
        .setAllianceFlipping(allianceFlip);
  }

  /**
   * Generates a new PathingCommand to go to the given supplied position.
   *
   * @param supplier The supplier of the goal position.
   * @return A new PathingCommand.
   */
  public Command generateToPoseSupplierCommand(Supplier<Pose2d> supplier) {
    Supplier<Pose2d> finalSupplier=() -> getPoseForAlliance(supplier.get());
    if(isDifferential){
      return new ActiveConditionalCommand(new PathingCommand(
        finalSupplier,
        robotPose,
        drive,
        robotProfile,
        builder.build(),
        subsystem,
        translationTolerance,
        rotationTolerance), new PathingCommand(
        finalSupplier,
        robotPose,
        differentialRotationConsumer,
        robotProfile,
        builder.build(),
        subsystem,
        translationTolerance,
        rotationTolerance), ()->robotPose.get().getTranslation().getDistance(finalSupplier.get().getTranslation())>translationTolerance);
    }
    return new PathingCommand(
        finalSupplier,
        robotPose,
        drive,
        robotProfile,
        builder.build(),
        subsystem,
        translationTolerance,
        rotationTolerance);
  }
  
  /**
   * Generates a new PathingCommand to go to a pose.
   *
   * @param pose The goal position.
   * @return A new PathingCommand.
   */
  public Command generateToPoseCommand(Pose2d pose) {
    return generateToPoseSupplierCommand(() -> pose);
  }

  /**
   * Generates a new PathingCommand to go to a pose.
   *
   * @param x The goal x position in meters.
   * @param y The goal y position in meters.
   * @param t The goal rotation in meters.
   * @return A new PathingCommand.
   */
  public Command generateToPoseCommand(double x, double y, double t) {
    return generateToPoseCommand(new Pose2d(x, y, new Rotation2d(t)));
  }

  /**
   * Generates a new PathingCommand to go to a translation, ignoring rotation.
   *
   * @param trans The translation.
   * @return A new PathingCommand.
   */
  public Command generateToTranslationCommand(Translation2d trans) {
    return generateToPoseSupplierCommand(() -> new Pose2d(trans, robotPose.get().getRotation()));
  }

  /**
   * Generates a new PathingCommand to go to a translation, ignoring rotation.
   *
   * @param x The goal x position in meters.
   * @param y The goal y position in meters.
   * @return A new PathingCommand.
   */
  public Command generateToTranslationCommand(double x, double y) {
    return generateToPoseSupplierCommand(
        () -> new Pose2d(new Translation2d(x, y), robotPose.get().getRotation()));
  }

  /**
   * Generates a new PathingCommand to go to a specified distance from a reference point within a
   * range of angles. For example, this can be used to go to a flexible shooting position.
   *
   * @param point The point to reference for the distance it is from it.
   * @param distance Supplier of the goal distance from the reference point in meters.
   * @param offset The offset of the robot's rotation relative to the target. 0 is the front of the
   *     robot facing the target.
   * @param centerGoal The center angle in radians to reference for the range.
   * @param maxAngleOff The maximum acceptable angle value in radians to be off by from the
   *     centerGoal to be in range.
   * @return A new PathingCommand.
   */
  public Command generateToDistFromPointCommand(
      Translation2d point,
      Supplier<Double> distance,
      double offset,
      Rotation2d centerGoal,
      double maxAngleOff) {
    final Rotation2d maxAngle = centerGoal.plus(new Rotation2d(maxAngleOff));
    final Rotation2d minAngle = centerGoal.minus(new Rotation2d(maxAngleOff));
    return generateToPoseSupplierCommand(
        () -> {
          Translation2d max = new Translation2d(distance.get(), maxAngle);
          Translation2d min =
              new Translation2d(distance.get(), centerGoal.minus(new Rotation2d(maxAngleOff)));
          SmartDashboard.putNumber(
              "Current Distance from point", robotPose.get().getTranslation().getDistance(point));
          Translation2d delta = robotPose.get().getTranslation().minus(point);
          Translation2d bestPoint = delta.times(distance.get() / delta.getNorm());
          Rotation2d angleOff = centerGoal.minus(bestPoint.getAngle());
          if (Math.abs(angleOff.getRadians()) > maxAngleOff) {
            double maxDist = max.getDistance(bestPoint);
            double minDist = min.getDistance(bestPoint);
            if (maxDist < minDist) {
              return new Pose2d(max.plus(point), maxAngle.plus(new Rotation2d(Math.PI - offset)));
            } else {
              return new Pose2d(min.plus(point), minAngle.plus(new Rotation2d(Math.PI - offset)));
            }
          }
          return new Pose2d(
              delta.times(distance.get() / delta.getNorm()).plus(point),
              delta.getAngle().plus(new Rotation2d(Math.PI - offset)));
        });
  }

  /**
   * Generates a new PathingCommand to go to a distance from a reference point within a range of
   * angles. For example, this can be used to go to a flexible shooting position.
   *
   * @param point The point to reference for the distance it is from it.
   * @param distance The goal distance from the reference point in meters.
   * @param offset The offset of the robot's rotation relative to the target. 0 is the front of the
   *     robot facing the target.
   * @param centerGoal The center angle in radians to reference for the range.
   * @param maxAngleOff The maximum acceptable angle value in radians to be off by from the
   *     centerGoal to be in range.
   * @return A new PathingCommand.
   */
  public Command generateToDistFromPointCommand(
      Translation2d point,
      double distance,
      double offset,
      Rotation2d centerGoal,
      double maxAngleOff) {
    return generateToDistFromPointCommand(point, () -> distance, offset, centerGoal, maxAngleOff);
  }

  /**
   * Generates a new PathingCommand to go to a distance from a reference point. Does not include
   * angle limiting. For example, this can be used to go to a flexible shooting position.
   *
   * @param point The point to reference for the distance it is from it.
   * @param distance The goal distance from the reference point in meters.
   * @param offset The offset of the robot's rotation relative to the target. 0 is the front of the
   *     robot facing the target.
   * @return A new PathingCommand.
   */
  public Command generateToDistFromPointCommand(
      Translation2d point, double distance, double offset) {
    return generateToDistFromPointCommand(point, distance, offset, new Rotation2d(), Math.PI);
  }
}
