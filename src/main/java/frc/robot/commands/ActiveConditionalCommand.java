package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;

public class ActiveConditionalCommand extends Command {
  Command onTrue, onFalse;
  Supplier<Boolean> condition;
  boolean hasFinishedTrue=false;
  public ActiveConditionalCommand(Command onTrue, Command onFalse, Supplier<Boolean> condition) {
    this.onTrue = onTrue;
    this.onFalse = onFalse;
    this.condition = condition;
    for (Subsystem sub : onTrue.getRequirements()) {
      addRequirements(sub);
    }
    for (Subsystem sub : onFalse.getRequirements()) {
      addRequirements(sub);
    }
  }

  public void initialize() {
    onTrue.initialize();
    onFalse.initialize();
  }

  public void execute() {
    if (condition.get()&&!hasFinishedTrue) {
      onTrue.execute();
    } else {
      onFalse.execute();
      hasFinishedTrue=true;
    }
  }

  @Override
  public boolean isFinished() {
    if(onFalse.isFinished() && !condition.get())hasFinishedTrue=false;
    return (onTrue.isFinished() && condition.get()) || (onFalse.isFinished() && !condition.get());
  }
  @Override
  public void end(boolean interrupt){
    hasFinishedTrue=false;
  }
}
