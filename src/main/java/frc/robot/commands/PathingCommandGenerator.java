package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.robotprofile.RobotProfile;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;

public class PathingCommandGenerator {
  private RobotProfile robotProfile;
  private Supplier<Pose2d> robotPose;
  private Consumer<ChassisSpeeds> drive;
  private Pathfinder pathfinder;
  private Subsystem subsystem;
  private double translationTolerance = .05,
      rotationTolerance = Math.PI/32;
public PathingCommandGenerator(RobotProfile robotProfile, Supplier<Pose2d> robotPose, Consumer<ChassisSpeeds> drive, Subsystem subsystem) {
    this.robotProfile = robotProfile;
    this.robotPose = robotPose;
    this.drive = drive;
    this.subsystem = subsystem;
    setField(Field.CRESCENDO_2024);
}
public void setField(String name) {
    pathfinder =
        new PathfinderBuilder(Filesystem.getDeployDirectory() + "\\" + name)
            .setRobotLength(robotProfile.getLength())
            .setRobotWidth(robotProfile.getWidth())
            .build();
}
public void setField(Field field) {
    pathfinder =
        new PathfinderBuilder(field)
            .setRobotLength(robotProfile.getLength())
            .setRobotWidth(robotProfile.getWidth())
            .build();
  }
  public PathingCommand toPoint(double x,double y,double t){
    return new PathingCommand(()->new Pose2d(x, y, new Rotation2d(t)), robotPose, drive, robotProfile, pathfinder, subsystem).setTolerances(translationTolerance, rotationTolerance);
  }
}
