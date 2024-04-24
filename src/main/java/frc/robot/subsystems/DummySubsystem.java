package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Annotations.Log;
import monologue.Logged;

public class DummySubsystem extends SubsystemBase implements Logged {
  @Log.NT double x = 0;

  public DummySubsystem() {}

  public Command up() {
    return this.run(() -> x++);
  }

  public Command down() {
    return this.run(() -> x--);
  }

  public Command zero() {
    return this.run(() -> x = 0);
  }
}
