package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ActiveConditionalCommand extends Command{
    Command onTrue,onFalse;
    Supplier<Boolean> condition;
    public ActiveConditionalCommand(Command onTrue, Command onFalse, Supplier<Boolean> condition) {
        this.onTrue = onTrue;
        this.onFalse = onFalse;
        this.condition = condition;
        for(Subsystem sub:onTrue.getRequirements()){
            addRequirements(sub);
        }
        for(Subsystem sub:onFalse.getRequirements()){
            addRequirements(sub);
        }
    }
    public void initialize(){
        onTrue.initialize();
        onFalse.initialize();
    }
    public void execute(){
        if(condition.get()){
            onTrue.execute();
        }else{
            onFalse.execute();
        }
    }
    @Override
    public boolean isFinished(){
        return (onTrue.isFinished()&&condition.get())||(onFalse.isFinished()&&!condition.get());
    }
}