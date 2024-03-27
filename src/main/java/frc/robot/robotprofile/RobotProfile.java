package frc.robot.robotprofile;

public class RobotProfile {
  private double maxVelocity,
      maxAcceleration,
      maxRotationalVelocity,
      maxRotationalAcceleration,
      length,
      width;
  private double safteyMultiplier = .95;

  public RobotProfile(
      double maxVelocity,
      double maxAcceleration,
      double maxRotationalVelocity,
      double maxRotationalAcceleration,
      double length,
      double width) {
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    this.maxRotationalVelocity = maxRotationalVelocity;
    this.maxRotationalAcceleration = maxRotationalAcceleration;
    this.length = length;
    this.width = width;
  }

  public RobotProfile(
      double robotMass,
      double wheelDiameter,
      double trackLength,
      double wheelBase,
      Motor driveMotor) {
    this.length = trackLength;
    this.width = wheelBase;

    maxVelocity = driveMotor.getFreeSpeed() * Math.PI * wheelDiameter / 60;
    maxAcceleration =
        driveMotor.getRealStallTorque()
            / (wheelDiameter / 2)
            / robotMass
            * 4; // We have four swerve modules

    double robotRadius = 2 * Math.max(trackLength, wheelBase) / Math.sqrt(2);
    maxRotationalVelocity = maxVelocity / robotRadius;
    maxRotationalAcceleration = maxAcceleration / robotRadius;
  }

  public double getMaxVelocity() {
    return maxVelocity * safteyMultiplier;
  }

  public void setMaxVelocity(double maxVelocity) {
    this.maxVelocity = maxVelocity;
  }

  public double getMaxAcceleration() {
    return maxAcceleration * safteyMultiplier;
  }

  public void setMaxAcceleration(double maxAcceleration) {
    this.maxAcceleration = maxAcceleration;
  }

  public double getMaxRotationalVelocity() {
    return maxRotationalVelocity * safteyMultiplier;
  }

  public void setMaxRotationalVelocity(double maxRotationalVelocity) {
    this.maxRotationalVelocity = maxRotationalVelocity;
  }

  public double getMaxRotationalAcceleration() {
    return maxRotationalAcceleration * safteyMultiplier;
  }

  public void setMaxRotationalAcceleration(double maxRotationalAcceleration) {
    this.maxRotationalAcceleration = maxRotationalAcceleration;
  }

  public double getLength() {
    return length;
  }

  public void setLength(double trackLength) {
    this.length = trackLength;
  }

  public double getWidth() {
    return width;
  }

  public void setWidth(double wheelBase) {
    this.width = wheelBase;
  }

  public double getSafteyMultiplier() {
    return safteyMultiplier;
  }

  public RobotProfile setSafteyMultiplier(double safteyMultiplier) {
    this.safteyMultiplier = safteyMultiplier;
    return this;
  }

  @Override
  public String toString() {
    return "RobotProfile [maxVelocity="
        + maxVelocity
        + ", maxAcceleration="
        + maxAcceleration
        + ", maxRotationalVelocity="
        + maxRotationalVelocity
        + ", maxRotationalAcceleration="
        + maxRotationalAcceleration
        + ", length="
        + length
        + ", width="
        + width
        + ", safteyMultiplier="
        + safteyMultiplier
        + "]";
  }
}
