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
import frc.utils.Range;
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
  private boolean linearPhysics = false;
  private boolean isDifferential = false;


  /**
   * Constructs a PathingCommandGenerator to generate {@code PathingCommand}s on a holonomic chassis
   * with the given settings. Defaults to using the layout for the 2024 field. To use a custom
   * field, add String or Field as a last parameter in the constructor.
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
   * Constructs a PathingCommandGenerator to generate {@code PathingCommand}s on a holonomic chassis
   * with the given settings. Creates a custom field from the given json name. To use the default
   * field, leave out the String from the constructor. To use a custom field from the Field enum,
   * replace the String value with a field value.
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
   * Constructs a PathingCommandGenerator to generate {@code PathingCommand}s on a holonomic chassis
   * with the given settings. Creates a custom field from the given json name. To use the default
   * field, leave out the String from the constructor. To use a custom field from the Field enum,
   * replace the String value with a field value.
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
   * Please only use this if you know what you are doing. Constructs a PathingCommandGenerator to
   * generate {@code PathingCommand}s on a holonomic chassis with the given settings. This
   * constructor allows you to pass in a custom Pathfinder builder instead of one being created with
   * a field layout as the default, with a custom json, or with a Field value. The builder can later
   * be modified using {@link #getBuilder}).
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
   * Constructs a PathingCommandGenerator to generate {@code PathingCommand}s on a differential
   * chassis with the given settings. Defaults to using the layout for the 2024 field. To use a
   * custom field, add String or Field as a last parameter in the constructor.
   *
   * @param robotProfile The {@link RobotProfile} to be used by this command generator.
   * @param robotPose Supplier of the robot's current position as a {@link Pose2d}. For example, a
   *     reference to a {@code getPose()} method.
   * @param drive Consumer to drive the robot. Must take {@link DifferentialDriveWheelSpeeds} in
   *     meters per second
   * @param trackWidth The wheel to wheel distance on the drivetrain, in meters
   * @param subsystem The drive subsystem (so it can be required).
   */
  public PathingCommandGenerator(
      RobotProfile robotProfile,
      Supplier<Pose2d> robotPose,
      Consumer<DifferentialDriveWheelSpeeds> drive,
      double trackWidth,
      DifferentialOrientationMode reverseMode,
      Subsystem subsystem) {
    this(robotProfile, robotPose, drive, trackWidth, subsystem, Field.CRESCENDO_2024);
  }

  /**
   * Constructs a PathingCommandGenerator to generate {@code PathingCommand}s on a differential
   * chassis with the given settings. Creates a custom field from the given json name. To use the
   * default field, leave out the String from the constructor. To use a custom field from the Field
   * enum, replace the String value with a field value.
   *
   * @param robotProfile The {@link RobotProfile} to be used by this command generator.
   * @param robotPose Supplier of the robot's current position as a {@link Pose2d}. For example, a
   *     reference to a {@code getPose()} method.
   * @param drive Consumer to drive the robot. Must take {@link DifferentialDriveWheelSpeeds} in
   *     meters per second
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
   * Constructs a PathingCommandGenerator to generate {@code PathingCommand}s on a differential
   * chassis with the given settings. Creates a custom field from the given json name. To use the
   * default field, leave out the String from the constructor. To use a custom field from the Field
   * enum, replace the String value with a field value.
   *
   * @param robotProfile The {@link RobotProfile} to be used by this command generator.
   * @param robotPose Supplier of the robot's current position as a {@link Pose2d}. For example, a
   *     reference to a {@code getPose()} method.
   * @param drive Consumer to drive the robot. Must take {@link DifferentialDriveWheelSpeeds} in
   *     meters per second
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
    this(robotProfile, robotPose, drive, trackWidth, subsystem, new PathfinderBuilder(field));
  }

  /**
   * Only use this if you know what you are doing. Constructs a PathingCommandGenerator to generate
   * {@code PathingCommand}s on a differential chassis with the given settings. This constructor
   * allows you to pass in a custom Pathfinder builder instead of one being created with a field
   * layout as the default, with a custom json, or with a Field value. The builder can later be
   * modified using {@link #getBuilder}).
   *
   * @param robotProfile The {@link RobotProfile} to be used by this command generator.
   * @param robotPose Supplier of the robot's current position as a {@link Pose2d}. For example, a
   *     reference to a {@code getPose()} method.
   * @param drive Consumer to drive the robot. Must take {@link DifferentialDriveWheelSpeeds} in
   *     meters per second
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
  double rotationalVelocity;
  private DifferentialOrientationMode differentialOrientationMode=DifferentialOrientationMode.AUTOMATIC;
  public PathingCommandGenerator setDifferentialOrientationMode(DifferentialOrientationMode mode){
    this.differentialOrientationMode=mode;
    return this;
  }
  public void setDifferentialDrive(
      Consumer<DifferentialDriveWheelSpeeds> diffDrive, double trackWidth) {
        isDifferential = true;
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(trackWidth);
        drive =
            (ChassisSpeeds speeds) -> {
              double theta =
                  new Rotation2d(Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond)).minus(robotPose.get().getRotation()).getRadians();
              boolean reversed=false;
              if(differentialOrientationMode==DifferentialOrientationMode.REVERSE)reversed=true;
              if(differentialOrientationMode==DifferentialOrientationMode.AUTOMATIC){
                reversed=Math.abs(theta)>Math.PI/2;
              }
              if(reversed)theta=new Rotation2d(theta).plus(new Rotation2d(Math.PI)).getRadians();
                  double velocity =
                  Math.sqrt(
                      speeds.vxMetersPerSecond * speeds.vxMetersPerSecond
                          + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond);
              double linearVelocity = velocity * Math.cos(theta)*(reversed?-1:1);
              if (!differentialOrientationMode.equals(DifferentialOrientationMode.AUTOMATIC) && Math.abs(theta) > Math.PI / 2) linearVelocity = 0;
              double desiredRotationalVelocity = (velocity) * Math.sin(theta) * 2 / trackWidth;
              double rotationSign=Math.signum(desiredRotationalVelocity);
              desiredRotationalVelocity=Math.abs(desiredRotationalVelocity);
              rotationalVelocity=Math.min(desiredRotationalVelocity,robotProfile.getMaxRotationalAcceleration()*.02+Math.abs(rotationalVelocity))*rotationSign;
              SmartDashboard.putNumber("Rotation Vel", rotationalVelocity);
              DifferentialDriveWheelSpeeds output =
                  kinematics.toWheelSpeeds(new ChassisSpeeds(linearVelocity, 0, rotationalVelocity));
              diffDrive.accept(output);
            };
        differentialRotationConsumer =
            (ChassisSpeeds speeds) -> {
              diffDrive.accept(
                  kinematics.toWheelSpeeds(new ChassisSpeeds(0, 0, speeds.omegaRadiansPerSecond)));
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
        .setAllianceFlipping(allianceFlip)
        .setPhysicsAlgorithmType(linearPhysics)
        .setDifferentialOrientationMode(differentialOrientationMode);
  }

  /**
   * Sets the physics algorithm used in path following. The linear algorithm propogates the current
   * velocity out along the path, starting at the robot, until it comes accross a point at which it
   * needs to slow down. A non-linear algorithm looks ahead and propogates velocities from points
   * that will need to be slowed down on(turns). This yeilds more accurate results and allows
   * tighter path following, but runs slower than the linear version.
   *
   * @param linear If the physics algorithm will search for slowdowns from the robot's perspective
   *     or will look ahead. A true value for this will run faster but yeild less accurate results
   *     than false;
   * @return This generator
   */
  public PathingCommandGenerator setPhysicsAlgorithmType(boolean linear) {
    this.linearPhysics = linear;
    return this;
  }

  /**
   * Generates a new PathingCommand to go to the given supplied position.
   *
   * @param supplier The supplier of the goal position.
   * @return A new PathingCommand.
   */
  public Command generateToPoseSupplierCommand(Supplier<Pose2d> supplier) {
    Supplier<Pose2d> finalSupplier = () -> getPoseForAlliance(supplier.get());
    if (isDifferential) {
      return new ActiveConditionalCommand(
          new PathingCommand(
              finalSupplier,
              robotPose,
              drive,
              robotProfile,
              builder.build(),
              subsystem,
              translationTolerance,
              rotationTolerance,
              linearPhysics),
          new PathingCommand(
              () -> new Pose2d(robotPose.get().getTranslation(), finalSupplier.get().getRotation()),
              robotPose,
              differentialRotationConsumer,
              robotProfile,
              builder.build(),
              subsystem,
              translationTolerance,
              rotationTolerance,
              linearPhysics),
          () ->
              robotPose.get().getTranslation().getDistance(finalSupplier.get().getTranslation())
                  > translationTolerance);
    }
    return new PathingCommand(
        finalSupplier,
        robotPose,
        drive,
        robotProfile,
        builder.build(),
        subsystem,
        translationTolerance,
        rotationTolerance,
        linearPhysics);
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
   * Generates a new PathingCommand to go to a supplied distance from a reference point within a
   * range of angles. For example, this can be used to go to a flexible shooting position or
   * gamepiece pickup from various angles.
   *
   * @param point The point to reference for the distance it is from it.
   * @param distance Supplier of the goal distance from the reference point in meters.
   * @param offset The offset of the robot's rotation relative to the target. 0 is the front of the
   *     robot facing the target.
   * @param centerGoal The center of the angle range to reference for the range.
   * @param maxAngleOff The maximum acceptable angle value to be off by from the centerGoal to be in
   *     range.
   * @return A new PathingCommand.
   */
  public Command generateToDistFromPointCommand(
      Translation2d point,
      Supplier<Double> distance,
      Rotation2d offset,
      Rotation2d centerGoal,
      Rotation2d maxAngleOff) {
    final Rotation2d maxAngle = centerGoal.plus(maxAngleOff);
    final Rotation2d minAngle = centerGoal.minus(maxAngleOff);
    return generateToPoseSupplierCommand(
        () -> {
          Translation2d max = new Translation2d(distance.get(), maxAngle);
          Translation2d min = new Translation2d(distance.get(), centerGoal.minus(maxAngleOff));
          SmartDashboard.putNumber(
              "Current Distance from point", robotPose.get().getTranslation().getDistance(point));
          Translation2d delta = robotPose.get().getTranslation().minus(point);
          Translation2d bestPoint = delta.times(distance.get() / delta.getNorm());
          Rotation2d angleOff = centerGoal.minus(bestPoint.getAngle());
          if (Math.abs(angleOff.getRadians()) > maxAngleOff.getRadians()) {
            double maxDist = max.getDistance(bestPoint);
            double minDist = min.getDistance(bestPoint);
            if (maxDist < minDist) {
              return new Pose2d(
                  max.plus(point), maxAngle.plus(new Rotation2d(Math.PI - offset.getRadians())));
            } else {
              return new Pose2d(
                  min.plus(point), minAngle.plus(new Rotation2d(Math.PI - offset.getRadians())));
            }
          }
          return new Pose2d(
              delta.times(distance.get() / delta.getNorm()).plus(point),
              delta.getAngle().plus(new Rotation2d(Math.PI - offset.getRadians())));
        });
  }

  /**
   * Generates a new PathingCommand to go to a set distance from a reference point within a range of
   * angles. For example, this can be used to go to a flexible shooting position or gamepiece pickup
   * from various angles.
   *
   * @param point The point to reference for the distance it is from it.
   * @param distance The goal distance from the reference point in meters.
   * @param offset The offset of the robot's rotation relative to the target. 0 is the front of the
   *     robot facing the target. Pi is the back of the robot facing the target.
   * @param centerGoal The center of the angle range to reference for the range.
   * @param maxAngleOff The maximum acceptable angle value to be off by from the centerGoal to be in
   *     range.
   * @return A new PathingCommand.
   */
  public Command generateToDistFromPointCommand(
      Translation2d point,
      double distance,
      Rotation2d offset,
      Rotation2d centerGoal,
      Rotation2d maxAngleOff) {
    return generateToDistFromPointCommand(point, () -> distance, offset, centerGoal, maxAngleOff);
  }

  /**
   * Generates a new PathingCommand to go to a supplied distance from a reference point. For
   * example, this can be used to go to a flexible shooting position or gamepiece pickup from
   * various angles.
   *
   * @param point The point to reference for the distance it is from it.
   * @param distance A supplier for he goal distance from the reference point in meters.
   * @param offset The offset of the robot's rotation relative to the target. 0 is the front of the
   *     robot facing the target. Pi is the back of the robot facing the target.
   * @return A new PathingCommand.
   */
  public Command generateToDistFromPointCommand(
      Translation2d point, Supplier<Double> distance, Rotation2d offset) {
    return generateToDistFromPointCommand(
        point, distance, offset, new Rotation2d(), new Rotation2d(Math.PI));
  }

  /**
   * Generates a new PathingCommand to go to a set distance from a reference point. For example,
   * this can be used to go to a flexible shooting position or gamepiece pickup from various angles.
   *
   * @param point The point to reference for the distance it is from it.
   * @param distance The goal distance from the reference point in meters.
   * @param offset The offset of the robot's rotation relative to the target. 0 is the front of the
   *     robot facing the target. Pi is the back of the robot facing the target.
   * @return A new PathingCommand.
   */
  public Command generateToDistFromPointCommand(
      Translation2d point, double distance, Rotation2d offset) {
    return generateToDistFromPointCommand(
        point, distance, offset, new Rotation2d(), new Rotation2d(Math.PI));
  }

  /**
   * Generates a new PathingCommand to go to a supplied distance from a reference point within a
   * range of angles. For example, this can be used to go to a flexible shooting position or
   * gamepiece pickup from various angles.
   *
   * @param point The point to reference for the distance it is from it.
   * @param distance Supplier of the goal distance from the reference point in meters.
   * @param centerGoal The center of the angle range to reference for the range.
   * @param maxAngleOff The maximum acceptable angle value to be off by from the centerGoal to be in
   *     range.
   * @return A new PathingCommand.
   */
  public Command generateToDistFromPointCommand(
      Translation2d point,
      Supplier<Double> distance,
      Rotation2d centerGoal,
      Rotation2d maxAngleOff) {
    return generateToDistFromPointCommand(
        point, distance, new Rotation2d(), centerGoal, maxAngleOff);
  }

  /**
   * Generates a new PathingCommand to go to a set distance from a reference point within a range of
   * angles. For example, this can be used to go to a flexible shooting position or gamepiece pickup
   * from various angles.
   *
   * @param point The point to reference for the distance it is from it.
   * @param distance The goal distance from the reference point in meters.
   * @param centerGoal The center of the angle range to reference for the range.
   * @param maxAngleOff The maximum acceptable angle value to be off by from the centerGoal to be in
   *     range.
   * @return A new PathingCommand.
   */
  public Command generateToDistFromPointCommand(
      Translation2d point, double distance, Rotation2d centerGoal, Rotation2d maxAngleOff) {
    return generateToDistFromPointCommand(
        point, () -> distance, new Rotation2d(), centerGoal, maxAngleOff);
  }

  /**
   * Generates a new PathingCommand to go to a supplied distance from a reference point. For
   * example, this can be used to go to a flexible shooting position or gamepiece pickup from
   * various angles.
   *
   * @param point The point to reference for the distance it is from it.
   * @param distance A supplier for he goal distance from the reference point in meters.
   * @return A new PathingCommand.
   */
  public Command generateToDistFromPointCommand(
      Translation2d point,
      Supplier<Double> distance) {
    return generateToDistFromPointCommand(point, distance, new Rotation2d(), new Rotation2d(), new Rotation2d(Math.PI));
  }

   /**
   * Generates a new PathingCommand to go to a set distance from a reference point. For example,
   * this can be used to go to a flexible shooting position or gamepiece pickup from various angles.
   *
   * @param point The point to reference for the distance it is from it.
   * @param distance The goal distance from the reference point in meters.
   * @return A new PathingCommand.
   */
  public Command generateToDistFromPointCommand(Translation2d point, double distance) {
    return generateToDistFromPointCommand(
        point, distance, new Rotation2d(), new Rotation2d(), new Rotation2d(Math.PI));
  }

  /**
   * Generates a new PathingCommand to go to a range of distances from a reference point within a
   * range of angles. For example, this can be used to go to a flexible shooting position or
   * gamepiece pickup from various angles.
   *
   * @param point The point to reference for the distance it is from it.
   * @param distance A range of the goal distances from the reference point in meters.
   * @param offset The offset of the robot's rotation relative to the target. 0 is the front of the
   *     robot facing the target.
   * @param centerGoal The center of the angle range to reference for the range.
   * @param maxAngleOff The maximum acceptable angle value to be off by from the centerGoal to be in
   *     range.
   * @return A new PathingCommand.
   */
  public Command generateToDistFromPointCommand(
      Translation2d point,
      Range distance,
      Rotation2d offset,
      Rotation2d centerGoal,
      Rotation2d maxAngleOff) {
    return generateToDistFromPointCommand(
        point,
        () -> distance.nearestValue(robotPose.get().getTranslation().getDistance(point)),
        offset,
        centerGoal,
        maxAngleOff);
  }

  /**
   * Generates a new PathingCommand to go to a supplied distance from a reference point. For
   * example, this can be used to go to a flexible shooting position or gamepiece pickup from
   * various angles.
   *
   * @param point The point to reference for the distance it is from it.
   * @param distance A range of the goal distances from the reference point in meters.
   * @param offset The offset of the robot's rotation relative to the target. 0 is the front of the
   *     robot facing the target. Pi is the back of the robot facing the target.
   * @return A new PathingCommand.
   */
  public Command generateToDistFromPointCommand(
      Translation2d point, Range distance, Rotation2d offset) {
    return generateToDistFromPointCommand(
        point, distance, offset, new Rotation2d(), new Rotation2d(Math.PI));
  }

  /**
   * Generates a new PathingCommand to go to a range of distances from a reference point within a
   * range of angles. For example, this can be used to go to a flexible shooting position or
   * gamepiece pickup from various angles.
   *
   * @param point The point to reference for the distance it is from it.
   * @param distance Range of the goal distances from the reference point in meters.
   * @param centerGoal The center of the angle range to reference for the range.
   * @param maxAngleOff The maximum acceptable angle value to be off by from the centerGoal to be in
   *     range.
   * @return A new PathingCommand.
   */
  public Command generateToDistFromPointCommand(
      Translation2d point, Range distance, Rotation2d centerGoal, Rotation2d maxAngleOff) {
    return generateToDistFromPointCommand(
        point, distance, new Rotation2d(), centerGoal, maxAngleOff);
  }

  /**
   * Generates a new PathingCommand to go to a range of distances from a reference point. For
   * example, this can be used to go to a flexible shooting position or gamepiece pickup from
   * various angles.
   *
   * @param point The point to reference for the distance it is from it.
   * @param distance A range of the goal distances from the reference point in meters.
   * @return A new PathingCommand.
   */
  public Command generateToDistFromPointCommand(Translation2d point, Range distance) {
    return generateToDistFromPointCommand(
        point, distance, new Rotation2d(), new Rotation2d(), new Rotation2d(Math.PI));
  }

  public static enum DifferentialOrientationMode{
    FORWARD,REVERSE,AUTOMATIC
  }
}