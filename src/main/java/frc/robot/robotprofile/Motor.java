package frc.robot.robotprofile;

public class Motor {
    private double maxStallTorque,realStallTorque,freeSpeed;

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
    public Motor gear(double gearRatio){
        freeSpeed/=gearRatio;
        maxStallTorque*=gearRatio;
        realStallTorque*=gearRatio;
        return this;
    }
}
