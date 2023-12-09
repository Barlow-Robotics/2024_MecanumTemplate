// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import frc.robot.Constants.*;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand ;

import java.io.FileWriter;
import java.util.ArrayList;
import java.util.HashMap;
import edu.wpi.first.networktables.*;
import java.util.Map;

import java.lang.Runnable ;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("PMD.ExcessiveImports")
public class RobotContainer {
    
    /****************** CONSTANTS ******************/

    /* Logitech Dual Action */
    public static final int LDALeftJoystickX = 0;
    public static final int LDALeftJoystickY = 1;
    public static final int LDARightJoystickX = 2;
    public static final int LDARightJoystickY = 3;

    public static final int LDALeftTrigger = 7;
    public static final int LDARightTrigger = 8;
    public static final int LDALeftBumper = 5;
    public static final int LDARightBumper = 6;
    public static final int LDAButtonA = 2;
    public static final int LDAButtonB = 3;
    public static final int LDAButtonX = 1;
    public static final int LDAButtonY = 4;
    public static final int LDABackButton = 9;
    public static final int LDAStartButton = 10;
    public static final int LDALeftJoystick = 11;
    public static final int LDARightJoystick = 12;

    public static final double LDAForwardAxisAttenuation = -0.5;
    public static final double LDALateralAxisAttenuation = 0.5;
    public static final double LDAYawAxisAttenuation = 0.5;

    /* RadioMaster TX12 */
    public static final int RMLeftGimbalX = 0;
    public static final int RMLeftGimbalY = 1;
    public static final int RMRightGimbalX = 3;
    public static final int RMRightGimbalY = 2;

    public static final int RMSliderF = 5;
    public static final int RMSliderE = 4;
    public static final int RMSliderC = 6;

    public static final int RMButtonD = 2;
    public static final int RMButtonA = 1;

    public static final double FowardAxisAttenuation = 1.0;
    public static final double LateralAxisAttenuation = 1.0;
    public static final double YawAxisAttenuation = 0.6;

    /* Xbox */
    public static final int XLeftStickX = 0;
    public static final int XLeftStickY = 1;
    public static final int XLeftTrigger = 2;
    public static final int XRightTrigger = 4;
    public static final int XRightStickX = 4;
    public static final int XRightStickY = 5;

    public static final int XButtonA = 1;
    public static final int XButtonB = 2;
    public static final int XButtonX = 3;
    public static final int XButtonY = 4;
    public static final int XLeftBumper = 5;
    public static final int XRightBumper = 6;
    public static final int XBackButton = 7;
    public static final int XStartButton = 8;
    public static final int XLeftStick = 9;
    public static final int XRightStick = 10;
    public static final int XWindowButton = 7;

    public static final double XForwardAxisAttenuation = -0.5;
    public static final double XLateralAxisAttenuation = 0.5;
    public static final double XYawAxisAttenuation = 0.5;

    /***********************************************/

    /* Subsystems */
    private final Drive driveSub = new Drive();

    /* Commands */

    /* Controllers */
    Joystick driverController; // controller 1
    Joystick operatorController; // controller 2

    /* Buttons & Axes */
   
    /* Shuffleboard */

    public RobotContainer() {
        configureButtonBindings();

        driveSub.setDefaultCommand(
            new DriveRobot(
                driveSub, driverController, RMLeftGimbalY, RMLeftGimbalX, RMRightGimbalX)
        );
    }

    private void configureButtonBindings() {
        /* Set Controller IDs */
        if (driverController == null) {
            System.out.println("null controller. Using joystick 2");
            driverController = new Joystick(1);
        }

        if (operatorController == null) {
            System.out.println("null controller. Using joystick 2");
            operatorController = new Joystick(2);
        }

        /* Initialize Buttons */

        /* Map Buttons to Commands */
    }

    public Command getAutonomousCommand() {
        return null;
    }
}