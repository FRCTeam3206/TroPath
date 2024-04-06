package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.robotprofile.RobotProfile;
import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.structures.ImpossiblePathException;
import me.nabdev.pathfinding.structures.Path;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;

/** A command to go to the given position. */
public class PathingCommand extends Command {
  /**
   * Statically configured robot profile that will be used by default when a new PathingCommand is
   * created.
   */
  private static RobotProfile defaultRobotProfile;

  /** The robot profile for this instance of the PathingCommand. */
  private RobotProfile robotProfile;

  /** Supplies the robot's current position. */
  private static Supplier<Pose2d> robotPose;

  /** Consumer to drive the robot given ChassisSpeeds. */
  private static Consumer<ChassisSpeeds> drive;

  /** The Pathfinder that is used for generating the paths. */
  private static Pathfinder pathfinder;

  private double velocity, rotationalVelocity = 0;
  private TrapezoidProfile translationProfile, rotationProfile;

  /** The goal position to end at. */
  private Pose2d goalPose;

  /**
   * A field that shows the next goal pose to be reached in the process of getting to the final one.
   */
  private Field2d nextPoseFieldDisplay = new Field2d();

  /** A field that shows the final goal pose. */
  private Field2d finalPoseFieldDisplay = new Field2d();

  /** The default amount this can be off by when considered to be complete. */
  private static double defaultTranslationTolerance = .05, defaultRotationTolerance = Math.PI / 32;

  /**
   * The amount this can be off by for this instance of PathingCommand when considered to be
   * complete.
   */
  private double translationTolerance = defaultTranslationTolerance,
      rotationTolerance = defaultTranslationTolerance;

  /** The drive subsystem. */
  private static Subsystem subsystem;

  private static final double dT = .02, eps = 1E-4;

  /**
   * Constructs a PathingCommand to go to the given position.
   *
   * @param pose The goal position.
   * @throws NullPointerException If the robot profile, pose supplier, drive speed consumer, or
   *     drive subsystem is null. Please call {@link PathingCommand#setRobot} and {@link
   *     PathingCommand#setDefaultRobotProfile} before constructing a PathingCommand.
   */
  public PathingCommand(Pose2d pose) {
    if (defaultRobotProfile == null)
      throw new NullPointerException(
          "Default Robot Profile is null, please call PathingCommand.setDefaultRobotProfile before this constructor");
    if (robotPose == null)
      throw new NullPointerException(
          "Robot Pose supplier is null. Please call PathingCommand.setRobot before this constructor");
    if (drive == null)
      throw new NullPointerException(
          "Drive Speed consumer is null. Please call PathingCommand.setRobot before this constructor");
    if (subsystem == null)
      throw new NullPointerException(
          "Drive Subsystem is null. Please call PathingCommand.setRobot before this constructor");
    setTolerances(defaultTranslationTolerance, defaultRotationTolerance);
    this.goalPose = pose;
    this.robotProfile = defaultRobotProfile;
    this.addRequirements(subsystem);
    setRobotProfile(defaultRobotProfile);
    SmartDashboard.putData("Next Pose", nextPoseFieldDisplay);
    SmartDashboard.putData("Final Pose", finalPoseFieldDisplay);
  }

  /**
   * Constructs a pathing command to go to the given position.
   *
   * @param x The goal x position.
   * @param y The goal y position.
   * @param rot The goal rotation.
   * @throws NullPointerException If the robot profile, pose supplier, drive speed consumer, or
   *     drive subsystem is null. Please call {@link PathingCommand#setRobot} and {@link
   *     PathingCommand#setDefaultRobotProfile} before constructing a PathingCommand.
   */
  public PathingCommand(double x, double y, double rot) {
    this(new Pose2d(x, y, new Rotation2d(rot)));
  }

  /**
   * Sets a different {@link RobotProfile} for this command than the configured default. To set the
   * default robot profile, use {@link PathingCommand#setDefaultRobotProfile}
   *
   * @param profile The robot profile to set.
   */
  public PathingCommand setRobotProfile(RobotProfile profile) {
    this.robotProfile = profile;
    translationProfile =
        new TrapezoidProfile(
            new Constraints(profile.getMaxVelocity(), profile.getMaxAcceleration()));
    rotationProfile =
        new TrapezoidProfile(
            new Constraints(
                profile.getMaxRotationalVelocity(), profile.getMaxRotationalAcceleration()));
    return this;
  }

  /**
   * Configures the robot to be able to be referenced by this command.
   *
   * @param robotPose Supplier of robot pose. This should usually be a reference to a getPose()
   *     method.
   * @param drive Consumer to drive the robot. Must take ChassisSpeeds and be field relative. This
   *     should usually be a reference to a drive() method.
   * @param subsystem The drive subsystem (so it can be required).
   */
  public static void setRobot(
      Supplier<Pose2d> robotPose, Consumer<ChassisSpeeds> drive, Subsystem subsystem) {
    PathingCommand.drive = drive;
    PathingCommand.robotPose = robotPose;
    PathingCommand.subsystem = subsystem;
  }

  /**
   * Sets the default {@link RobotProfile} to be used when a new PathingCommand is constructed.
   *
   * @param profile The robot profile to set as the default.
   */
  public static void setDefaultRobotProfile(RobotProfile robotProfile) {
    PathingCommand.defaultRobotProfile = robotProfile;
    pathfinder =
        new PathfinderBuilder(Field.CRESCENDO_2024)
            .setRobotLength(robotProfile.getLength())
            .setRobotWidth(robotProfile.getWidth())
            .build();
  }

  /**
   * @return The default {@link RobotProfile} that has been configured.
   */
  public static RobotProfile getDefaultRobotProfile() {
    return defaultRobotProfile;
  }

  /**
   * Sets the field/obstacle layout to a custom one from a json. The layout defaults to the 2024
   * field.
   *
   * @param name The name the custom field json file, which must be located in the deploy folder.
   *     NOT the full path.
   */
  public static void setCustomField(String name) {
    pathfinder =
        new PathfinderBuilder(Filesystem.getDeployDirectory() + "\\" + name)
            .setRobotLength(defaultRobotProfile.getLength())
            .setRobotWidth(defaultRobotProfile.getWidth())
            .build();
  }

  /**
   * Sets the field/obstacle layout to one from the {@link Field} enum. The layout defaults to the
   * 2024 field.
   *
   * @param field The enum value of the desired field.
   */
  public static void setCustomField(Field field) {
    pathfinder =
        new PathfinderBuilder(field)
            .setRobotLength(defaultRobotProfile.getLength())
            .setRobotWidth(defaultRobotProfile.getWidth())
            .build();
  }

  public void execute() {
    finalPoseFieldDisplay.setRobotPose(goalPose);
    double deltaRotation;
    deltaRotation = robotPose.get().getRotation().minus(goalPose.getRotation()).getRadians();
    rotationalVelocity =
        rotationProfile.calculate(
                dT,
                new TrapezoidProfile.State(deltaRotation, rotationalVelocity),
                new TrapezoidProfile.State(0, 0))
            .velocity;
    Path path = null;
    long start = System.currentTimeMillis();
    try {
      path = pathfinder.generatePath(robotPose.get(), goalPose);
      SmartDashboard.putNumber("Path generation time", System.currentTimeMillis() - start);
    } catch (ImpossiblePathException e) {
      e.printStackTrace();
      return;
    }
    Pose2d nextTargetPose;
    Pose2d usedPose;
    if (path.size() <= 1) {
      nextTargetPose = goalPose;
      usedPose = robotPose.get();
    } else {
      usedPose = path.get(0).asPose2d();
      nextTargetPose = path.get(1).asPose2d();
    }
    nextPoseFieldDisplay.setRobotPose(
        new Pose2d(nextTargetPose.getTranslation(), goalPose.getRotation()));
    double dX = nextTargetPose.getX() - robotPose.get().getX(),
        dY = nextTargetPose.getY() - robotPose.get().getY();
    double total = Math.abs(dX) + Math.abs(dY);
    TrapezoidProfile.State nextState;
    start = System.currentTimeMillis();
    if (path.size() <= 1) {
      nextState =
          new TrapezoidProfile.State(
              usedPose.getTranslation().getDistance(goalPose.getTranslation()), 0);
    } else {
      nextState = getNextState(path);
    }
    SmartDashboard.putNumber("Physics Time", System.currentTimeMillis() - start);
    velocity =
        translationProfile.calculate(dT, new TrapezoidProfile.State(0, velocity), nextState)
            .velocity;
    SmartDashboard.putNumber("Velocity", velocity);
    double xSpeed = dX / total * velocity;
    double ySpeed = dY / total * velocity;

    drive.accept(new ChassisSpeeds(xSpeed, ySpeed, rotationalVelocity));
  }

  private TrapezoidProfile.State getNextState(Path path) {
    Pose2d lastPose = path.get(0).asPose2d();

    ArrayList<Pose2d> poses = path.asPose2dList();
    double cumulativeDistance = 0;
    for (int i = 1; i < poses.size() - 1; i++) {
      Pose2d currentPose = poses.get(i);
      Pose2d nextPose = poses.get(i + 1);
      double nextDistance = nextPose.getTranslation().getDistance(currentPose.getTranslation());
      if (cumulativeDistance
          > robotProfile.getMaxVelocity()
              * robotProfile.getMaxVelocity()
              / robotProfile.getMaxAcceleration()
              / 2) {
        cumulativeDistance += nextDistance;
        continue; // Don't do extra math, just find the distance of the path
      }
      double angle = angle(lastPose, currentPose, nextPose);

      if (angle < eps) continue;
      double stopDist = nextDistance / angle;
      double maxAllowedVelocity = Math.sqrt(stopDist * 2 * robotProfile.getMaxAcceleration());
      if (maxAllowedVelocity < robotProfile.getMaxVelocity()) {
        return new TrapezoidProfile.State(cumulativeDistance, maxAllowedVelocity);
      }
      cumulativeDistance += nextDistance;
      lastPose = currentPose;
    }
    return new TrapezoidProfile.State(
        cumulativeDistance
            + poses
                .get(poses.size() - 1)
                .getTranslation()
                .getDistance(poses.get(poses.size() - 2).getTranslation()),
        0);
  }

  private double angle(Pose2d pose1, Pose2d pose2, Pose2d pose3) {
    double d1 = pose1.getTranslation().getDistance(pose2.getTranslation());
    double d2 = pose2.getTranslation().getDistance(pose3.getTranslation());
    double d3 = pose3.getTranslation().getDistance(pose1.getTranslation());
    return Math.PI - Math.acos((d1 * d1 + d2 * d2 - d3 * d3) / (2 * d1 * d2));
  }

  /**
   * Sets the tolerances to something different than the default. Should be used if this particular
   * PathingCommand should have different tolerances than the others. The tolerances are the maximum
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
  public PathingCommand setTolerances(double translationTolerance, double rotationTolerance) {
    this.translationTolerance = translationTolerance;
    this.rotationTolerance = rotationTolerance;
    return this;
  }

  /**
   * Sets the default tolerances to be used when a new pathing command is constructed. The
   * tolerances are the maximum allowed error for which the robot is considered to have reached the
   * goal and should be tuned to your robot. They should be as small as possible without being more
   * precise than the robot can achieve well. If the tolerance is too small, the robot will spend
   * longer than it should trying to get perfectly in position (and move the wheels in different
   * directions as it tries to perfectly adjust). If it is at a good amount, it should stop as soon
   * as it reaches the position (the wheels moving back and forth should not be noticeable).
   *
   * @param translationTolerance The translation tolerance to set. In meters. Defaults to 5 cm.
   * @param rotationTolerance The rotation tolerance to set. In radians. Defaults to pi/32.
   */
  public static void setDefaultTolerances(double translationTolerance, double rotationTolerance) {
    PathingCommand.defaultTranslationTolerance = translationTolerance;
    PathingCommand.defaultRotationTolerance = rotationTolerance;
  }

  public boolean isFinished() {
    return (robotPose.get().getTranslation().getDistance(goalPose.getTranslation())
                    + velocity * velocity / 2 / robotProfile.getMaxAcceleration()
                < translationTolerance
            && Math.abs(robotPose.get().getRotation().minus(goalPose.getRotation()).getRadians())
                    + rotationalVelocity
                        * rotationalVelocity
                        / 2
                        / robotProfile.getMaxRotationalAcceleration()
                < rotationTolerance);
  }
}
