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

public class PathingCommand extends Command {
  private static RobotProfile defaultRobotProfile;
  private RobotProfile robotProfile;
  private static Supplier<Pose2d> robotPose;
  private static Consumer<ChassisSpeeds> drive;
  private static Pathfinder pathfinder;
  private double velocity, rotationalVelocity = 0;
  private TrapezoidProfile translationProfile, rotationProfile;
  private Pose2d goalPose;
  private Field2d nextPoseFieldDisplay = new Field2d();
  private Field2d finalPoseFieldDisplay = new Field2d();
  private boolean continnuous = false;
  private static double defaultTranslationTolerance=.05, defaultRotationTolerance=Math.PI/32;
  private double translationTolerance = defaultTranslationTolerance, rotationTolerance = defaultTranslationTolerance;
  private static Subsystem subsystem;

  public PathingCommand(Pose2d pose) {
    if(defaultRobotProfile==null)throw new NullPointerException("Default Robot Profile is null, please call PathingCommand.setDefaultRobotProfile before this constructor");
    if(robotPose==null)throw new NullPointerException("Robot Pose supplier is null. Please call PathingCommand.set Robot before this constructor");
    if(drive==null)throw new NullPointerException("Drive Speed consumer is null. Please call PathingCommand.set Robot before this constructor");
    if(subsystem==null)throw new NullPointerException("Drive Subsystem is null. Please call PathingCommand.set Robot before this constructor");
    setTolerances(defaultTranslationTolerance, defaultRotationTolerance);
    this.goalPose = pose;
    this.robotProfile = defaultRobotProfile;
    this.addRequirements(subsystem);
    setRobotProfile(defaultRobotProfile);
    SmartDashboard.putData("Next Pose", nextPoseFieldDisplay);
    SmartDashboard.putData("Final Pose", finalPoseFieldDisplay);
  }

  public PathingCommand(double x, double y, double rot) {
    this(new Pose2d(x, y, new Rotation2d(rot)));
  }

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

  public static void setRobot(
      Supplier<Pose2d> robotPose,Consumer<ChassisSpeeds> drive, Subsystem subsystem) {
    PathingCommand.drive = drive;
    PathingCommand.robotPose = robotPose;
    PathingCommand.subsystem = subsystem;
  }

  public static void setDefaultRobotProfile(RobotProfile robotProfile) {
    PathingCommand.defaultRobotProfile = robotProfile;
    pathfinder =
        new PathfinderBuilder(Field.CHARGED_UP_2023)
            .setRobotLength(robotProfile.getLength())
            .setRobotWidth(robotProfile.getWidth())
            .build();
  }

  public static RobotProfile getDefaultRobotProfile() {
    return defaultRobotProfile;
  }

  public static void setCustomField(String name) {
    pathfinder =
        new PathfinderBuilder(Filesystem.getDeployDirectory() + "\\" + name)
            .setRobotLength(defaultRobotProfile.getLength())
            .setRobotWidth(defaultRobotProfile.getWidth())
            .build();
  }

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
                .02,
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
        translationProfile.calculate(.02, new TrapezoidProfile.State(0, velocity), nextState)
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

      if (angle < 1E-4) continue;
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

  public PathingCommand setContinnuous(boolean continnuous) {
    this.continnuous = continnuous;
    return this;
  }

  public PathingCommand setTolerances(double translationTolerance, double rotationTolerance) {
    this.translationTolerance = translationTolerance;
    this.rotationTolerance = rotationTolerance;
    return this;
  }
  public static void setDefaultTolerances(double translationTolerance, double rotationTolerance){
    PathingCommand.defaultTranslationTolerance=translationTolerance;
    PathingCommand.defaultRotationTolerance=rotationTolerance;
  }
  public boolean isFinished() {
    // If continnuous true, always returns false
    // Otherwise returns true if done(the auto stop) is true or the tolerances are met
    return !continnuous
        && (robotPose.get().getTranslation().getDistance(goalPose.getTranslation())+velocity*velocity/2/robotProfile.getMaxAcceleration()
                < translationTolerance
            && Math.abs(robotPose.get().getRotation().minus(goalPose.getRotation()).getRadians())+rotationalVelocity*rotationalVelocity/2/robotProfile.getMaxRotationalAcceleration()
                < rotationTolerance);
  }
}
