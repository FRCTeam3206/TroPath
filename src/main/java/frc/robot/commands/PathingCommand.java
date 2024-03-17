package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robotprofile.RobotProfile;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.structures.ImpossiblePathException;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;

public class PathingCommand extends Command{
    private static RobotProfile robotProfile;
    private static Supplier<Pose2d> robotPose;
    private static Consumer<Transform2d> drive;
    Pathfinder pathfinder;
    TrajectoryConfig config = new TrajectoryConfig(1 /* Max vel */, 9999 /* Max accel */);
    double velocity,rotationalVelocity=0;
    TrapezoidProfile translationProfile,rotationProfile;
    double maxVelocity,maxAcceleration;
    double stoppingDistAllowance=0;
    boolean finish=false;
    Pose2d pose;
    public PathingCommand(Pose2d pose){
        this.pose=pose;
        this.maxVelocity=robotProfile.getMaxVelocity();
        this.maxAcceleration=robotProfile.getMaxAcceleration();
        translationProfile=new TrapezoidProfile(new Constraints(maxVelocity, maxAcceleration));
        rotationProfile=new TrapezoidProfile(new Constraints(robotProfile.getMaxRotationalVelocity(), robotProfile.getMaxRotationalAcceleration()));
        pathfinder=new PathfinderBuilder(Field.CHARGED_UP_2023).setRobotLength(robotProfile.getTrackLength()).setRobotWidth(robotProfile.getWheelBase()).setCornerDist(Math.sqrt(maxVelocity*maxVelocity+maxVelocity*maxVelocity/maxAcceleration/maxAcceleration)).build();
    }
    public static void setRobot(Supplier<Pose2d> robotPose,Consumer<Transform2d> drive){
        PathingCommand.robotPose=robotPose;
        PathingCommand.drive=drive;
    }
    public static void setRobotProfile(RobotProfile robotProfile) {
        PathingCommand.robotProfile = robotProfile;
    }
    public static RobotProfile getRobotProfile(){
        return robotProfile;
    }
    public PathingCommand setStoppingDistAllowance(double stoppingDistAllowance){
        this.stoppingDistAllowance=stoppingDistAllowance;
        return this;
    }
    boolean done=false;
    public void execute(){
        double deltaRotation;
        deltaRotation = robotPose.get().getRotation().minus(pose.getRotation()).getRadians();
        rotationalVelocity=rotationProfile.calculate(.02, new TrapezoidProfile.State(deltaRotation,rotationalVelocity), new TrapezoidProfile.State(0,0)).velocity;
        Trajectory path=null;
        try {
                path = pathfinder.generateTrajectory(robotPose.get(), pose, config);
        } catch (ImpossiblePathException e) {
            drive.accept(new Transform2d(0, 0, new Rotation2d(rotationalVelocity)));
            velocity=0;
            done=rotationalVelocity<1E-4&&finish;
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
          

          
    
          drive.accept(new Transform2d(xSpeed,ySpeed,new Rotation2d(rotationalVelocity)));
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
        return done;
    }
}
