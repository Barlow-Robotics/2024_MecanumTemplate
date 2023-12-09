// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;

public class DriveRobot extends Command {

    Drive driveSub;
    Joystick driverController;
    int controllerFwdID;
    int controllerLatID;
    int controllerYawID;

    private double FwdRateLimit = 7;
    SlewRateLimiter fwdInputRamp = new SlewRateLimiter(FwdRateLimit);
    private double LatRateLimit = 7;
    SlewRateLimiter latInputRamp = new SlewRateLimiter(LatRateLimit);
    private double YawRateLimit = 7;
    SlewRateLimiter yawInputRamp = new SlewRateLimiter(YawRateLimit);

    private double DeadBand = 0.01;

    private double FwdSpeedAttenuation = 0.5;
    private double LatSpeedAttenuation = 0.5;
    private double YawAttenuation = 0.5;

    private float yawMultiplier = 1.0f;

    public PIDController pid;

    public DriveRobot(Drive driveSub, Joystick driverController, int fwdID, int latID, int yawID) {
        this.driveSub = driveSub;
        this.driverController = driverController;
        this.controllerFwdID = fwdID;
        this.controllerLatID = latID;
        this.controllerYawID = yawID;

        addRequirements(driveSub);
    }

    @Override
    public void initialize() {
        pid.reset();
    }

    @Override
    public void execute() {

        /* Forward Speed */
        double fwdSpeed = fwdInputRamp.calculate(driverController.getRawAxis(controllerFwdID));
        if (Math.abs(fwdSpeed) < DeadBand) {
            fwdSpeed = 0.0;
        }
        fwdSpeed *= FwdSpeedAttenuation;

        NetworkTableInstance.getDefault().getEntry("driverController/fwdRawAxis")
                .setDouble(driverController.getRawAxis(controllerFwdID));
        NetworkTableInstance.getDefault().getEntry("driverController/fwdRampedAxis")
                .setDouble(fwdInputRamp.calculate(driverController.getRawAxis(controllerFwdID)));
        NetworkTableInstance.getDefault().getEntry("driverController/fwdSpeed")
                .setDouble(fwdSpeed);


        /* Lateral Speed */
        double latSpeed = latInputRamp.calculate(driverController.getRawAxis(controllerLatID));
        if (Math.abs(latSpeed) < DeadBand) {
            latSpeed = 0.0;
        }
        latSpeed *= LatSpeedAttenuation;

        NetworkTableInstance.getDefault().getEntry("driverController/latRampedAxis")
                .setDouble(driverController.getRawAxis(controllerLatID));
        NetworkTableInstance.getDefault().getEntry("driverController/latRampedAxis")
                .setDouble(latInputRamp.calculate(driverController.getRawAxis(controllerLatID)));
        NetworkTableInstance.getDefault().getEntry("driverController/latSpeed")
                .setDouble(latSpeed);

        /* Yaw Speed */
        double yawSpeed = yawInputRamp.calculate(driverController.getRawAxis(controllerYawID));
        if (Math.abs(yawSpeed) < DeadBand) {
            yawSpeed = 0.0;
        }
        yawSpeed *= YawAttenuation;

        NetworkTableInstance.getDefault().getEntry("driverController/yawRawAxis")
                .setDouble(driverController.getRawAxis(controllerYawID));
        NetworkTableInstance.getDefault().getEntry("driverController/yawRampedAxis")
                .setDouble(yawInputRamp.calculate(driverController.getRawAxis(controllerYawID)));
        NetworkTableInstance.getDefault().getEntry("driverController/yawSpeed")
                .setDouble(yawSpeed);

        driveSub.drive(fwdSpeed, latSpeed, rawSpeed, false);
    }

    @Override
    public void end(boolean interrupted) {
        driveSub.drive(0.0, 0.0, 0.0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
