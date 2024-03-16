package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.structures.ImpossiblePathException;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;

public class PathingCommand extends Command{
    Pathfinder pathfinder = new PathfinderBuilder(Field.CHARGED_UP_2023).build();
    TrajectoryConfig config = new TrajectoryConfig(1 /* Max vel */, 9999 /* Max accel */);
    DriveSubsystem drive;
    double velocity=0;
    TrapezoidProfile profile=new TrapezoidProfile(new Constraints(3, 1));
    public PathingCommand(DriveSubsystem drive){
        this.drive=drive;

    }
    public void execute(){
        Trajectory myPath=null;
        try {
            myPath = pathfinder.generateTrajectory(drive.getPose(), new Pose2d(8, 4, new Rotation2d()), config);
        } catch (ImpossiblePathException e) {
            return;
        }
          State state=myPath.getStates().get(0);
          State nextState=myPath.getStates().get(1);
          double dX=nextState.poseMeters.getX()-state.poseMeters.getX();
          double dY=nextState.poseMeters.getY()-state.poseMeters.getY();
          double total=Math.abs(dX)+Math.abs(dY);
          velocity=profile.calculate(.02, new TrapezoidProfile.State(0,velocity), new TrapezoidProfile.State(myPath.getTotalTimeSeconds(),0)).velocity;
          double xSpeed=dX/total*velocity;
          double ySpeed=dY/total*velocity;
          drive.drive(xSpeed, ySpeed, 0, true, false);
    }
}
