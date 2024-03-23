package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robotprofile.RobotProfile;

import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.structures.ImpossiblePathException;
import me.nabdev.pathfinding.structures.Path;
import me.nabdev.pathfinding.structures.Vertex;
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
  private Field2d nextPose=new Field2d();
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
    SmartDashboard.putData(nextPose);
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
    Path path = null;
    try {
      path = pathfinder.generatePath(robotPose.get(), pose);
    } catch (ImpossiblePathException e) {
      e.printStackTrace();
      return;
    }
    Pose2d nextTargetPose;
    Pose2d usedPose;
    if(path.size()<=1){
      nextTargetPose=pose;
      usedPose=robotPose.get();
    }else{
      usedPose=path.get(0).asPose2d();
      nextTargetPose=path.get(1).asPose2d();
    }
    System.out.println(robotPose.get());
    Transform2d delta=nextTargetPose.minus(usedPose);
    nextPose.setRobotPose(nextTargetPose);
    double dX=delta.getX(),dY=delta.getY();
    SmartDashboard.putNumber("Move dX", dX);
    SmartDashboard.putNumber("Move dY", dY);
    double total = Math.abs(dX) + Math.abs(dY);
    TrapezoidProfile.State nextState;
    if(path.size()<=1){
      nextState=new TrapezoidProfile.State(usedPose.getTranslation().getDistance(pose.getTranslation()), 0);
    }else{
      nextState=getNextState(path);
    }
    velocity =
        translationProfile.calculate(
                .02, new TrapezoidProfile.State(0, velocity), nextState)
            .velocity;
    SmartDashboard.putNumber("Velocity",velocity);
    double xSpeed = dX / total * velocity;
    double ySpeed = dY / total * velocity;

    drive.accept(new Transform2d(xSpeed, ySpeed, new Rotation2d(rotationalVelocity)));
  }

  private TrapezoidProfile.State getNextState(Path path) {
    Pose2d lastPose=path.get(0).asPose2d();
    int i=0;
    ArrayList<Pose2d> poses=path.asPose2dList();
    double cumulativeDistance=0;
    for (Pose2d pose : poses.subList(0, poses.size()-1)) {
      Pose2d nextPose=poses.get(i+1);
      double nextDistance=nextPose.getTranslation().getDistance(pose.getTranslation());
      if(cumulativeDistance>robotProfile.getMaxVelocity()*robotProfile.getMaxVelocity()/robotProfile.getMaxAcceleration()/2){
        cumulativeDistance+=nextDistance;
        continue;//Don't do extra math, just find the distance of the path
      }
      double angle=angle(lastPose, pose, nextPose);
      if(i==0)SmartDashboard.putNumber("Angle",angle);
      double stopDist = nextDistance*Math.PI/angle/2;
      double maxAllowedVelocity =
          Math.sqrt(stopDist*2*robotProfile.getMaxAcceleration());
      // if (maxAllowedVelocity < robotProfile.getMaxVelocity()) {
      //   return new TrapezoidProfile.State(cumulativeDistance, maxAllowedVelocity);
      // }
      
      cumulativeDistance+=nextDistance;
      lastPose=pose;
      i++;
    }
    return new TrapezoidProfile.State(
        cumulativeDistance+poses.get(poses.size()-1).getTranslation().getDistance(poses.get(poses.size()-2).getTranslation()),
        0);
  }
  private double angle(Pose2d pose1, Pose2d pose2, Pose2d pose3){
    double d1=pose1.getTranslation().getDistance(pose2.getTranslation());
    double d2=pose2.getTranslation().getDistance(pose3.getTranslation());
    double d3=pose3.getTranslation().getDistance(pose1.getTranslation());
    return Math.PI-Math.acos((d1*d1+d3*d3-d2*d2)/(2*d1*d3));
  }
  public boolean isFinished() {
    return done;
  }
}
