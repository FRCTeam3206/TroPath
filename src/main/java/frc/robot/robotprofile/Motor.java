package frc.robot.robotprofile;

public class Motor {
  private double maxStallTorque, realStallTorque, freeSpeed;

  public Motor(double maxStallTorque, double realStallTorque, double freeSpeed) {
    this.maxStallTorque = maxStallTorque;
    this.realStallTorque = realStallTorque;
    this.freeSpeed = freeSpeed;
  }

  public double getMaxStallTorque() {
    return maxStallTorque;
  }

  public void setMaxStallTorque(double maxStallTorque) {
    this.maxStallTorque = maxStallTorque;
  }

  public double getRealStallTorque() {
    return realStallTorque;
  }

  public void setRealStallTorque(double realStallTorque) {
    this.realStallTorque = realStallTorque;
  }

  public double getFreeSpeed() {
    return freeSpeed;
  }

  public void setFreeSpeed(double freeSpeed) {
    this.freeSpeed = freeSpeed;
  }

  public Motor gear(double gearRatio) {
    freeSpeed /= gearRatio;
    maxStallTorque *= gearRatio;
    realStallTorque *= gearRatio;
    return this;
  }

  public static final double SDS_L1 = 8.14,
      SDS_L2 = 6.75,
      SDS_L3 = 6.12,
      SDS_L4 = 5.14,
      REV_LOW = 5.5,
      REV_MED = 5.08,
      REV_HIGH = 4.71,
      REV_EHIGH_1 = 4.5,
      REV_EHIGH_2 = 4.29,
      REV_EHIGH_3 = 4.00,
      REV_EHIGH_4 = 3.75,
      REV_EHIGH_5 = 3.56,
      WCP_X1_LOW=7.85,
      WCP_X1_MED=7.13,
      WCP_X1_HIGH=6.54,
      WCP_X2_LOW=6.56,
      WCP_X2_MED=5.96,
      WCP_X2_HIGH=5.46,
      WCP_X3_LOW=5.14,
      WCP_X3_MED=4.75,
      WCP_X3_HIGH=4.41,
      WCP_XS1_12=6.00,
      WCP_XS1_13=5.54,
      WCP_XS1_14=5.14,
      WCP_XS2_14=4.71,
      WCP_XS2_15=4.40,
      WCP_XS2_16=4.13;

  public static Motor CIM() {
    return new Motor(2.4, .686, 5333);
  }

  public static Motor NEO() {
    return new Motor(3.28, .701, 5880);
  }

  public static Motor FALCON() {
    return new Motor(4.69, .70003, 6380);
  }

  public static Motor VORTEX() {
    return new Motor(3.6, .621, 6784);
  }

  public static Motor KRAKENX60() {
    return new Motor(7.09, .746, 6000);
  }
}
