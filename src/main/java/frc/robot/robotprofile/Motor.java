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
      REV_EHIGH_5 = 3.56;
  public static Motor CIM(){return new Motor(2.4, .686, 5333);}
  public static Motor NEO(){return new Motor(3.28, .701, 5880);}
  public static Motor FALCON (){return new Motor(4.69, .70003, 6380);}
  public static Motor VORTEX (){return new Motor(3.6, .621, 6784);}
  public static Motor KRAKENX60 (){return new Motor(7.09, .746, 6000);}
}
