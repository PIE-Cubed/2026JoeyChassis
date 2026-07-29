// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;


/** Add your docs here. */
public class Wheels {
    private SparkFlex driveMotor;
    private SparkFlexConfig driveMotorConfig;
    private RelativeEncoder driveEncoder;
    private EncoderConfig driveEncoderConfig;
    private SparkMax rotateMotor;
    private SparkMaxConfig rotateMotorConfig;
    private SparkAbsoluteEncoder rotateEncoder;
    private AbsoluteEncoderConfig rotateEncoderConfig;


    public Wheels(int driveID, int rotateID, boolean invertDriveMotor) {
        driveMotor  = new SparkFlex(driveID, MotorType.kBrushless);
        driveMotorConfig = new SparkFlexConfig();
        driveMotorConfig.smartCurrentLimit(Robot.VORTEX_CURRENT_LIMIT);
        driveMotorConfig.idleMode(IdleMode.kBrake);
        driveMotorConfig.inverted(invertDriveMotor);

        rotateMotor = new SparkMax(rotateID, MotorType.kBrushless);
        rotateMotorConfig = new SparkMaxConfig();
        // Same thing as the drive motor configs calls
        rotateMotorConfig
            .smartCurrentLimit(Robot.NEO_550_CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();
        driveEncoderConfig = new EncoderConfig();

        rotateEncoder = rotateMotor.getAbsoluteEncoder();
        rotateEncoderConfig = new AbsoluteEncoderConfig();
    }

}
