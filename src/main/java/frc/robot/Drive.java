package frc.robot;

public class Drive {
    private Wheels frontLeft;
    private Wheels frontRight;
    private Wheels backLeft;
    private Wheels backRight;

    public Drive() {
        frontLeft =  new Wheels(0, 0, false);
        frontRight = new Wheels(0, 0, false);
        backLeft   = new Wheels(0, 0, false);
        backRight  = new Wheels(0, 0, false);
    }

    public void teleopDrive(double fwdPower, double strafePower, double rotatePower) { 
        
    }


}
