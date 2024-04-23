package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.robotprofile.RobotProfile;
import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.structures.ImpossiblePathException;
import me.nabdev.pathfinding.structures.Path;

/** A command to go to the given position. */
public class PathingCommand extends Command {
  protected RobotProfile robotProfile;
  protected Supplier<Pose2d> robotPose;
  protected Consumer<ChassisSpeeds> drive;
  protected Pathfinder pathfinder;
  protected double velocity, rotationalVelocity = 0;
  protected TrapezoidProfile translationProfile, rotationProfile;
  protected Supplier<Pose2d> goalPoseSupplier;
  protected double translationTolerance = .05, rotationTolerance = Math.PI / 32;
  protected Field2d nextPoseFieldDisplay = new Field2d();
  protected Field2d finalPoseFieldDisplay = new Field2d();
  protected static final double dT = .02, eps = 1E-4;
  protected PathProfiler pathProfiler;
  protected boolean linearPhysics = false;

  /**
   * Constructs a PathingCommand. This method is called by the {@link PathingCommandGenerator}.
   * Please use this generator to make a PathingCommand.
   */
  public PathingCommand(
      Supplier<Pose2d> goalSupplier,
      Supplier<Pose2d> currentPoseSupplier,
      Consumer<ChassisSpeeds> drive,
      RobotProfile robotProfile,
      Pathfinder pathfinder,
      Subsystem subsystem,
      double translationTolerance,
      double rotationTolerance,
      boolean linearPhysics) {
    this.goalPoseSupplier = goalSupplier;
    this.robotPose = currentPoseSupplier;
    this.drive = drive;
    this.linearPhysics = linearPhysics;
    setRobotProfile(robotProfile);
    this.pathfinder = pathfinder;
    this.addRequirements(subsystem);
    SmartDashboard.putData("Next Pose", nextPoseFieldDisplay);
    SmartDashboard.putData("Final Pose", finalPoseFieldDisplay);
  }

  private PathingCommand setRobotProfile(RobotProfile profile) {
    this.robotProfile = profile;
    translationProfile =
        new TrapezoidProfile(
            new Constraints(profile.getMaxVelocity(), profile.getMaxAcceleration()));
    rotationProfile =
        new TrapezoidProfile(
            new Constraints(
                profile.getMaxRotationalVelocity(), profile.getMaxRotationalAcceleration()));
    pathProfiler = new PathProfiler(profile.getMaxVelocity(), profile.getMaxAcceleration());
    return this;
  }

  public void execute() {
    finalPoseFieldDisplay.setRobotPose(goalPoseSupplier.get());
    double deltaRotation;
    deltaRotation =
        robotPose.get().getRotation().minus(goalPoseSupplier.get().getRotation()).getRadians();
    rotationalVelocity =
        rotationProfile.calculate(
                dT,
                new TrapezoidProfile.State(deltaRotation, rotationalVelocity),
                new TrapezoidProfile.State(0, 0))
            .velocity;
    Path path = null;
    long start = System.currentTimeMillis();
    try {
      path = pathfinder.generatePath(robotPose.get(), goalPoseSupplier.get());
      SmartDashboard.putNumber("Path generation time", System.currentTimeMillis() - start);
    } catch (ImpossiblePathException e) {
      e.printStackTrace();
      return;
    }
    Pose2d nextTargetPose;
    Pose2d usedPose;
    if (path.size() <= 1) {
      nextTargetPose = goalPoseSupplier.get();
      usedPose = robotPose.get();
    } else {
      usedPose = path.get(0).asPose2d();
      nextTargetPose = path.get(1).asPose2d();
    }
    nextPoseFieldDisplay.setRobotPose(
        new Pose2d(nextTargetPose.getTranslation(), goalPoseSupplier.get().getRotation()));
    double dX = nextTargetPose.getX() - robotPose.get().getX(),
        dY = nextTargetPose.getY() - robotPose.get().getY();
    TrapezoidProfile.State nextState;
    start = System.currentTimeMillis();
    if (linearPhysics) {
      if (path.size() <= 1) {
        nextState =
            new TrapezoidProfile.State(
                usedPose.getTranslation().getDistance(goalPoseSupplier.get().getTranslation()), 0);
      } else {
        nextState = getNextState(path);
      }

      velocity =
          translationProfile.calculate(dT, new TrapezoidProfile.State(0, velocity), nextState)
              .velocity;
    } else {
      velocity = pathProfiler.getNextRobotSpeed(velocity, robotPose.get(), path.asPose2dList());
    }
    SmartDashboard.putNumber("Physics Time", System.currentTimeMillis() - start);
    SmartDashboard.putNumber("Velocity", velocity);
    double theta=Math.atan2(dY, dX);
    double xSpeed = velocity*Math.cos(theta);
    double ySpeed = velocity*Math.sin(theta);
    if (Double.isNaN(xSpeed) || Double.isNaN(ySpeed)) {
      xSpeed = 0;
      ySpeed = 0;
    }
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

  public boolean isFinished() {
    return (robotPose.get().getTranslation().getDistance(goalPoseSupplier.get().getTranslation())
            < translationTolerance
        && Math.abs(
                    robotPose
                        .get()
                        .getRotation()
                        .minus(goalPoseSupplier.get().getRotation())
                        .getRadians())
            < rotationTolerance);
  }
}