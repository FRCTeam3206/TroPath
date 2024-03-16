package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.structures.ImpossiblePathException;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;

public class PathingCommand extends Command{
    Pathfinder pathfinder;
    TrajectoryConfig config = new TrajectoryConfig(1 /* Max vel */, 9999 /* Max accel */);
    DriveSubsystem drive;
    double velocity=0;
    TrapezoidProfile profile;
    double maxVelocity,maxAcceleration;
    public PathingCommand(DriveSubsystem drive, double maxVelocity, double maxAcceleration){
        this.drive=drive;
        profile=new TrapezoidProfile(new Constraints(maxVelocity, maxAcceleration));
        this.maxVelocity=maxVelocity;
        this.maxAcceleration=maxAcceleration;
        pathfinder=new PathfinderBuilder(Field.CHARGED_UP_2023).setRobotLength(.9).setRobotWidth(.9).setCornerDist(Math.sqrt(maxVelocity*maxVelocity+maxVelocity*maxVelocity/maxAcceleration/maxAcceleration)).build();
    }
    boolean done=false;
    public void execute(){
        Trajectory myPath=null;
        try {
            myPath = pathfinder.generateTrajectory(drive.getPose(), new Pose2d(8, 4, new Rotation2d()), config);
        } catch (ImpossiblePathException e) {
            System.out.println("No Path");
            drive.drive(0, 0, 0, true, false);
            velocity=0;
            SmartDashboard.putNumber("Velocity",0);
            done=true;
            return;
        }
          State state=myPath.getStates().get(0);
          State nextState=myPath.getStates().get(1);
          double dX=nextState.poseMeters.getX()-state.poseMeters.getX();
          double dY=nextState.poseMeters.getY()-state.poseMeters.getY();
          double total=Math.abs(dX)+Math.abs(dY);
          double totalDistance=myPath.getTotalTimeSeconds()-(myPath.getStates().get(myPath.getStates().size()-1).timeSeconds-myPath.getStates().get(myPath.getStates().size()-2).timeSeconds);
          velocity=profile.calculate(.02, new TrapezoidProfile.State(0,velocity), new TrapezoidProfile.State(totalDistance,0)).velocity;
          
          SmartDashboard.putNumber("Distance Left",totalDistance);
          SmartDashboard.putNumber("Velocity",velocity);
          double xSpeed=dX/total*velocity;
          double ySpeed=dY/total*velocity;
          drive.driveSpeed(xSpeed, ySpeed, 0, true, false);
    }
    public boolean isFinished(){
        return done;
    }
}
