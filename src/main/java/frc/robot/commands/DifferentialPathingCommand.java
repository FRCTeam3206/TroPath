package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.robotprofile.RobotProfile;
import java.util.function.Consumer;
import java.util.function.Supplier;
import me.nabdev.pathfinding.Pathfinder;

public class DifferentialPathingCommand extends PathingCommand {
  private Consumer<ChassisSpeeds> linearDrive, rotationalDrive;

  public DifferentialPathingCommand(
      Supplier<Pose2d> goalSupplier,
      Supplier<Pose2d> currentPoseSupplier,
      Consumer<ChassisSpeeds> linearDrive,
      Consumer<ChassisSpeeds> rotationalDrive,
      RobotProfile robotProfile,
      Pathfinder pathfinder,
      Subsystem subsystem,
      double translationTolerance,
      double rotationTolerance,
      boolean linearPhysics) {
    super(
        goalSupplier,
        currentPoseSupplier,
        linearDrive,
        robotProfile,
        pathfinder,
        subsystem,
        translationTolerance,
        rotationTolerance,
        linearPhysics);
    this.linearDrive = linearDrive;
    this.rotationalDrive = rotationalDrive;
  }

  @Override
  public void execute() {
    if (this.robotPose.get().getTranslation().getDistance(goalPoseSupplier.get().getTranslation())
        < translationTolerance) {
      this.drive = rotationalDrive;
    } else {
      this.drive = linearDrive;
    }
    super.execute();
  }
}
