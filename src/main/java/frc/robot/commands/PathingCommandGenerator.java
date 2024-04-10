package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.robotprofile.RobotProfile;
import frc.utils.AllianceUtil;
import java.util.function.Consumer;
import java.util.function.Supplier;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;

public class PathingCommandGenerator {
  private RobotProfile robotProfile;
  private Supplier<Pose2d> robotPose;
  private Consumer<ChassisSpeeds> drive;
  private Pathfinder pathfinder;
  private Subsystem subsystem;
  private double translationTolerance = .05, rotationTolerance = Math.PI / 32;
  private boolean allianceFlip = true;

  public PathingCommandGenerator(
      RobotProfile robotProfile,
      Supplier<Pose2d> robotPose,
      Consumer<ChassisSpeeds> drive,
      Subsystem subsystem) {
    this.robotProfile = robotProfile;
    this.robotPose = robotPose;
    this.drive = drive;
    this.subsystem = subsystem;
    AllianceUtil.setRobot(robotPose);
    setField(Field.CRESCENDO_2024);
  }

  private Pose2d getPoseForAlliance(Pose2d pose) {
    if (allianceFlip) return AllianceUtil.getPoseForAlliance(pose);
    return pose;
  }

  public PathingCommandGenerator setAllianceFlipping(boolean flag) {
    allianceFlip = flag;
    return this;
  }

  public PathingCommandGenerator setField(String name) {
    pathfinder =
        new PathfinderBuilder(Filesystem.getDeployDirectory() + "\\" + name)
            .setRobotLength(robotProfile.getLength())
            .setRobotWidth(robotProfile.getWidth())
            .build();
    return this;
  }

  public PathingCommandGenerator setField(Field field) {
    pathfinder =
        new PathfinderBuilder(field)
            .setRobotLength(robotProfile.getLength())
            .setRobotWidth(robotProfile.getWidth())
            .build();
    return this;
  }

  public PathingCommandGenerator setPathfinder(Pathfinder pathfinder) {
    this.pathfinder = pathfinder;
    return this;
  }

  /**
   * <p>Sets the tolerances for this PathingCommandGenerator. These default to 5 cm and pi / 32 radians.</p>
   * The tolerances are the maximum
   * allowed error for which the robot is considered to have reached the goal and should be tuned to
   * your robot. They should be as small as possible without being more precise than the robot can
   * achieve well. If the tolerance is too small, the robot will spend longer than it should trying
   * to get perfectly in position (and move the wheels in different directions as it tries to
   * perfectly adjust). If it is at a good amount, it should stop as soon as it reaches the position
   * (the wheels moving back and forth should not be noticeable).
   *
   * @param translationTolerance The translation tolerance to set. In meters. Defaults to 5 cm.
   * @param rotationTolerance The rotation tolerance to set. In radians. Defaults to pi/32.
   */
  public PathingCommandGenerator setTolerances(double translationTolerance, double rotationTolerance) {
    this.translationTolerance = translationTolerance;
    this.rotationTolerance = rotationTolerance;
    return this;
  }

  public PathingCommand toPoseSupplier(Supplier<Pose2d> supplier) {
    return new PathingCommand(
            () -> getPoseForAlliance(supplier.get()),
            robotPose,
            drive,
            robotProfile,
            pathfinder,
            subsystem)
        .setTolerances(translationTolerance, rotationTolerance);
  }

  public PathingCommand toPose(Pose2d pose) {
    return toPoseSupplier(() -> pose);
  }

  public PathingCommand toPoint(double x, double y, double t) {
    return toPose(new Pose2d(x, y, new Rotation2d(t)));
  }

  public PathingCommand toTranslation(Translation2d trans) {
    return toPoseSupplier(() -> new Pose2d(trans, robotPose.get().getRotation()));
  }

  public PathingCommand toTranslation(double x, double y) {
    return toPoseSupplier(() -> new Pose2d(new Translation2d(x, y), robotPose.get().getRotation()));
  }

  public PathingCommand toDistFromPoint(
      Translation2d point,
      double distance,
      double offset,
      Rotation2d centerGoal,
      double maxAngleOff) {
    final Rotation2d maxAngle = centerGoal.plus(new Rotation2d(maxAngleOff));
    final Rotation2d minAngle = centerGoal.minus(new Rotation2d(maxAngleOff));
    final Translation2d max = new Translation2d(distance, maxAngle);
    final Translation2d min =
        new Translation2d(distance, centerGoal.minus(new Rotation2d(maxAngleOff)));
    return toPoseSupplier(
        () -> {
          SmartDashboard.putNumber(
              "Current Distance from point", robotPose.get().getTranslation().getDistance(point));
          Translation2d delta = robotPose.get().getTranslation().minus(point);
          Translation2d bestPoint = delta.times(distance / delta.getNorm());
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
              delta.times(distance / delta.getNorm()).plus(point),
              delta.getAngle().plus(new Rotation2d(Math.PI - offset)));
        });
  }

  public PathingCommand toDistFromPoint(Translation2d point, double distance, double offset) {
    return toDistFromPoint(point, distance, offset, new Rotation2d(), Math.PI);
  }
}
