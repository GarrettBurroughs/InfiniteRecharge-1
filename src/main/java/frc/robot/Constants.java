/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.controller.PIDController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int kFrontLeftMotorPort = 7;
        public static final int kFrontRightMotorPort = 1;
        public static final int kBackLeftMotorPort = 9;
        public static final int kBackRightMotorPort = 3;

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterInches = 6;
        public static final double kEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;

        public static final double kTurnP = 1;
        public static final double kTurnI = 0;
        public static final double kTurnD = 0;

        public static final double kTurnToleranceDeg = 5;
        public static final double kTurnRateToleranceDegPerS = 10; // degrees per second

        public static final double ksVolts = 0.22; //TODO
        public static final double kvVoltSecondsPerMeter = 1.98; //TODO
        public static final double kaVoltSecondsSquaredPerMeter = 0.2; //TODO

        public static final double kPDriveVel = 8.5; //TODO

        public static final double kMaxSpeedMetersPerSecond = 3; //TODO
        public static final double kMaxAccelerationMetersPerSecondSquared = 3; //TODO

        public static final double kRamseteB = 2; //TODO
        public static final double kRamseteZeta = 0.7; //TODO

    }

    public static final class ShooterConstants {
        public static final int kShooterPort = 6;
    }

    public static final class TransportConstants {
        public static final int kTransportTopPort = 5;
        public static final int kTransportBotPort = 15;
    }

    public static final class IntakeConstants{
        public static final int kIntakeDeployLeftPort = 13;
        public static final int kIntakeDeployRightPort = 4;
        public static final int kIntakeWheelPort = 10; //TODO: TBD
        public static final int kIntakeDeployLimitSwitchPort = 0;
    }

    public static final class ControlPanelManipulatorConstants {
        public static final I2C.Port kColorSensorPort = I2C.Port.kMXP;
        public static final int kWheelMotorPort = 4;
    }

    public static final class VisionPIDConstants {
        public static final double kVisionTurnP = 1;
        public static final double kVisionTurnI = 0;
        public static final double kVisionTurnD = 0;

        public static final double kVisionDistP = 1;
        public static final double kVisionDistI = 0;
        public static final double kVisionDistD = 0;
    }

    public static final class IOConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kAuxiliaryControllerPort = 1;
    }
}