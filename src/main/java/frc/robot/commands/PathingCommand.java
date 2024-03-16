package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    double velocity,rotationalVelocity=0;
    TrapezoidProfile translationProfile,rotationProfile;
    double maxVelocity,maxAcceleration;
    double stoppingDistAllowance=0;
    Pose2d pose;
    public PathingCommand(Pose2d pose,DriveSubsystem drive, double maxVelocity, double maxAcceleration, double maxRotationalVelocity, double maxRotationalAcceleration){
        this.drive=drive;
        translationProfile=new TrapezoidProfile(new Constraints(maxVelocity, maxAcceleration));
        rotationProfile=new TrapezoidProfile(new Constraints(maxRotationalVelocity, maxRotationalAcceleration));
        this.maxVelocity=maxVelocity;
        this.maxAcceleration=maxAcceleration;
        this.pose=pose;
        pathfinder=new PathfinderBuilder(Field.CHARGED_UP_2023).setRobotLength(.9).setRobotWidth(.9).setCornerDist(Math.sqrt(maxVelocity*maxVelocity+maxVelocity*maxVelocity/maxAcceleration/maxAcceleration)).build();
    }
    public PathingCommand setStoppingDistAllowance(double stoppingDistAllowance){
        this.stoppingDistAllowance=stoppingDistAllowance;
        return this;
    }
    boolean done=false;
    public void execute(){
        double deltaRotation=drive.getPose().getRotation().minus(pose.getRotation()).getRadians();
        rotationalVelocity=rotationProfile.calculate(.02, new TrapezoidProfile.State(deltaRotation,rotationalVelocity), new TrapezoidProfile.State(0,0)).velocity;
        Trajectory path=null;
        try {
            path = pathfinder.generateTrajectory(drive.getPose(), pose, config);
        } catch (ImpossiblePathException e) {
            drive.drive(0, 0, rotationalVelocity, true, false);
            velocity=0;
            done=rotationalVelocity<1E-4;
            return;
        }
          State thisState=path.getStates().get(0);
          State nextState=path.getStates().get(1);
          double dX=nextState.poseMeters.getX()-thisState.poseMeters.getX();
          double dY=nextState.poseMeters.getY()-thisState.poseMeters.getY();
          double total=Math.abs(dX)+Math.abs(dY);
          velocity=translationProfile.calculate(.02, new TrapezoidProfile.State(0,velocity), getNextState(path)).velocity;
          getNextState(path);
          double xSpeed=dX/total*velocity;
          double ySpeed=dY/total*velocity;
          

          
    
          drive.driveSpeed(xSpeed, ySpeed, rotationalVelocity, true, false);
        }
    private TrapezoidProfile.State getNextState(Trajectory path){
        for(State state:path.getStates()){
            if(state.curvatureRadPerMeter<1E-4)continue;
            double stopDist=Math.PI/2/Math.abs(state.curvatureRadPerMeter)+stoppingDistAllowance;
            double maxAllowedVelocity=stopDist/Math.sqrt(1+Math.pow(maxAcceleration,-2));
            if(maxAllowedVelocity<maxVelocity){
                return new TrapezoidProfile.State(state.timeSeconds,maxAllowedVelocity);
            }
        }
        return new TrapezoidProfile.State(path.getTotalTimeSeconds()-(path.getStates().get(path.getStates().size()-1).timeSeconds-path.getStates().get(path.getStates().size()-2).timeSeconds), 0);
    }
    public boolean isFinished(){
        return false;
    }
}
