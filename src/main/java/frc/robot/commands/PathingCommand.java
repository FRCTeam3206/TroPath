package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robotprofile.RobotProfile;
import java.util.function.Consumer;
import java.util.function.Supplier;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.structures.ImpossiblePathException;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;

public class PathingCommand extends Command {
  private static RobotProfile robotProfile;
  private static Supplier<Pose2d> robotPose;
  private static Consumer<Transform2d> drive;
  private static Pathfinder pathfinder;
  private TrajectoryConfig config = new TrajectoryConfig(1 /* Max vel */, 9999 /* Max accel */);
  private double velocity, rotationalVelocity = 0;
  private TrapezoidProfile translationProfile, rotationProfile;
  private double stoppingDistAllowance = 0;
  private boolean finish = false;
  private Pose2d pose;
  private static double maxStopDist;

  public PathingCommand(Pose2d pose) {
    this.pose = pose;
    translationProfile =
        new TrapezoidProfile(
            new Constraints(robotProfile.getMaxVelocity(), robotProfile.getMaxAcceleration()));
    rotationProfile =
        new TrapezoidProfile(
            new Constraints(
                robotProfile.getMaxRotationalVelocity(),
                robotProfile.getMaxRotationalAcceleration()));
  }

  public static void setRobot(Supplier<Pose2d> robotPose, Consumer<Transform2d> drive) {
    PathingCommand.robotPose = robotPose;
    PathingCommand.drive = drive;
  }

  public static void setRobotProfile(RobotProfile robotProfile) {
    PathingCommand.robotProfile = robotProfile;
    maxStopDist =
        robotProfile.getMaxVelocity()*robotProfile.getMaxVelocity()/2/robotProfile.getMaxAcceleration();
    pathfinder =
        new PathfinderBuilder(Field.CHARGED_UP_2023)
            .setRobotLength(robotProfile.getLength())
            .setRobotWidth(robotProfile.getWidth())
            .setCornerDist(maxStopDist)
            .build();
  }

  public static RobotProfile getRobotProfile() {
    return robotProfile;
  }

  public PathingCommand setStoppingDistAllowance(double stoppingDistAllowance) {
    this.stoppingDistAllowance = stoppingDistAllowance;
    return this;
  }

  boolean done = false;

  public void execute() {
    double deltaRotation;
    deltaRotation = robotPose.get().getRotation().minus(pose.getRotation()).getRadians();
    rotationalVelocity =
        rotationProfile.calculate(
                .02,
                new TrapezoidProfile.State(deltaRotation, rotationalVelocity),
                new TrapezoidProfile.State(0, 0))
            .velocity;
    Trajectory path = null;
    try {
      path = pathfinder.generateTrajectory(robotPose.get(), pose, config);
    } catch (ImpossiblePathException e) {
      drive.accept(new Transform2d(0, 0, new Rotation2d(rotationalVelocity)));
      velocity = 0;
      done = rotationalVelocity < 1E-4 && finish;
      return;
    }
    State thisState = path.getStates().get(0);
    State nextState = path.getStates().get(1);
    double dX = nextState.poseMeters.getX() - thisState.poseMeters.getX();
    double dY = nextState.poseMeters.getY() - thisState.poseMeters.getY();
    SmartDashboard.putNumber("Move dX", dX);
    SmartDashboard.putNumber("Move dY", dY);
    double total = Math.abs(dX) + Math.abs(dY);
    velocity =
        translationProfile.calculate(
                .02, new TrapezoidProfile.State(0, velocity), getNextState(path))
            .velocity;
    getNextState(path);
    double xSpeed = dX / total * velocity;
    double ySpeed = dY / total * velocity;

    drive.accept(new Transform2d(xSpeed, ySpeed, new Rotation2d(rotationalVelocity)));
  }

  private TrapezoidProfile.State getNextState(Trajectory path) {

    for (State state : path.getStates()) {
      if (state.curvatureRadPerMeter < 1E-4) continue;
      double stopDist = 1 / Math.abs(state.curvatureRadPerMeter) + stoppingDistAllowance;
      double maxAllowedVelocity =
          Math.sqrt(stopDist/2/robotProfile.getMaxAcceleration());
      if (maxAllowedVelocity < robotProfile.getMaxVelocity()) {
        return new TrapezoidProfile.State(state.timeSeconds, maxAllowedVelocity);
      }
      if(state.timeSeconds>robotProfile.getMaxVelocity()*robotProfile.getMaxVelocity()/robotProfile.getMaxAcceleration()){
        break;
      }
    }
    return new TrapezoidProfile.State(
        path.getTotalTimeSeconds()
            - (path.getStates().get(path.getStates().size() - 1).timeSeconds
                - path.getStates().get(path.getStates().size() - 2).timeSeconds),
        0);
  }

  public boolean isFinished() {
    return done;
  }
}
