// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Controls {

    private ZorroController zorro;

    public Controls() {

        zorro = new ZorroController(0);
    
    }

    

    public double getForwardPowerFwdPositive() {
        double   fwdPower;

        fwdPower = zorro.getLeftY();
        //System.out.println("Forward Power:" + fwdPower);
        return fwdPower;
    }

    public double getStrafePowerLeftPositive() {
        double   strafePower;

        strafePower = zorro.getLeftX();
        //System.out.println("Strafe Power:" + strafePower);
        return strafePower * -1;
    }

    public double getRotateCCWPositive() {
        double   rotatePower;

        rotatePower = zorro.getRightX();
        //System.out.println("Rotate Power:" + rotatePower);
        return rotatePower * -1;
    }
}
