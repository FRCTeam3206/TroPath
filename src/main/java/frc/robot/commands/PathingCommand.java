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
import frc.utils.CommandDuringPath;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.function.Supplier;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.structures.ImpossiblePathException;
import me.nabdev.pathfinding.structures.Path;

/** A command to go to the given position. */
public class PathingCommand extends Command {
  private RobotProfile robotProfile;
  private Supplier<Pose2d> robotPose;
  private Consumer<ChassisSpeeds> drive;
  private Pathfinder pathfinder;
  private double velocity, rotationalVelocity = 0;
  private TrapezoidProfile rotationProfile;
  private Supplier<Pose2d> goalPoseSupplier;
  private double translationTolerance = .05, rotationTolerance = Math.PI / 32;
  private Field2d nextPoseFieldDisplay = new Field2d();
  private Field2d finalPoseFieldDisplay = new Field2d();
  private static final double dT = .02;
  private PathProfiler pathProfiler;
  private boolean linearPhysics = false;
  private ArrayList<CommandDuringPath> inactiveCommands = new ArrayList<CommandDuringPath>();
  private ArrayList<CommandDuringPath> activeCommands = new ArrayList<CommandDuringPath>();

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
    rotationProfile =
        new TrapezoidProfile(
            new Constraints(
                profile.getMaxRotationalVelocity(), profile.getMaxRotationalAcceleration()));
    pathProfiler = new PathProfiler(profile.getMaxVelocity(), profile.getMaxAcceleration());
    return this;
  }

  public void addCommandWithEndDist(Command command, double dist) {
    inactiveCommands.add(new CommandDuringPath(command, -1, dist));
  }

  public void addCommandWithStartDist(Command command, double startDist) {
    inactiveCommands.add(new CommandDuringPath(command, startDist, -1));
  }

  public void addCommandWithStartEndDist(Command command, double startDist, double endDist) {
    inactiveCommands.add(new CommandDuringPath(command, startDist, endDist)); // The constructor will hand mixed up start and end by assuming intention of correct order.
  }

  public void addCommandDistBasedCondition (Command command, Predicate<Double> isActive) {
    inactiveCommands.add(new CommandDuringPath(command, isActive));
  }

  private void checkInactiveCommands(double dist) {
    for (int i = 0; i < inactiveCommands.size();) {
      CommandDuringPath currentCommand = inactiveCommands.get(i);
      if (currentCommand.getIsActive(dist)) {
        currentCommand.getCommand().initialize();
        inactiveCommands.remove(i);
        activeCommands.add(currentCommand);
      } else {
        i++;
      }
    }
  }

  private void executeActiveCommands() {
    for (int i = 0; i < activeCommands.size(); i++) {
      activeCommands.get(i).getCommand().execute();
    }
  }

  private void checkActiveCommands(double dist) {
    for (int i = 0; i < activeCommands.size();) {
      CommandDuringPath currentCommand = activeCommands.get(i);
      if (!currentCommand.getIsActive(dist)) {
        currentCommand.getCommand().end(true);
        activeCommands.remove(i);
        inactiveCommands.add(currentCommand);
      } else if (currentCommand.getCommand().isFinished()) {
        currentCommand.getCommand().end(false);
        activeCommands.remove(i);
      } else {
        i++;
      }
    }
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
    if (path.size() <= 1) {
      nextTargetPose = goalPoseSupplier.get();
    } else {
      nextTargetPose = path.get(1).asPose2d();
    }
    nextPoseFieldDisplay.setRobotPose(
        new Pose2d(nextTargetPose.getTranslation(), goalPoseSupplier.get().getRotation()));
    double dX = nextTargetPose.getX() - robotPose.get().getX(),
        dY = nextTargetPose.getY() - robotPose.get().getY();
    start = System.currentTimeMillis();
    if (linearPhysics) {
      velocity=pathProfiler.nextVelocityLinear(velocity, path.asPose2dList());
    } else {
      velocity = pathProfiler.getNextRobotSpeed(velocity, path.asPose2dList());
    }
    checkInactiveCommands(pathProfiler.getDistanceToGoal());
    executeActiveCommands();
    checkActiveCommands(pathProfiler.getDistanceToGoal());
    SmartDashboard.putNumber("Distance To Goal",pathProfiler.getDistanceToGoal());
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

  public boolean isFinished() {
    return (robotPose.get().getTranslation().getDistance(goalPoseSupplier.get().getTranslation())
                + velocity * velocity / 2 / robotProfile.getMaxAcceleration()
            < translationTolerance
        && Math.abs(
                    robotPose
                        .get()
                        .getRotation()
                        .minus(goalPoseSupplier.get().getRotation())
                        .getRadians())
                + rotationalVelocity
                    * rotationalVelocity
                    / 2
                    / robotProfile.getMaxRotationalAcceleration()
            < rotationTolerance);
  }
}
