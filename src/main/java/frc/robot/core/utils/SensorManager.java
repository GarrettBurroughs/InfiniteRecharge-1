package frc.robot.core.utils;

import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.core.components.ControlSystem.DriverButtons;
import frc.robot.core.Robot;
import edu.wpi.first.wpilibj.util.Color;


public class SensorManager {

    private static Color colorSensorCalibrate_White, colorSensorCalibrate_Black;

    //KEY VALUES//
    public static class KeyColorSensorValues {

        public static double redValForY = 0.3;
        public static double greenValForY = 0.5;
        public static double redValForR = 0.4;
        public static double greenValForG = 0.5;
        public static double blueValForB = 0.4;

    }

    //RESETTERS//
    public static void resetSensors() {
<<<<<<< Updated upstream
        Hardware.navx.reset();
=======

        //Hardware.navx.reset();
>>>>>>> Stashed changes
        Hardware.m_colorSensor.calibrateFunctions(new Color(255,255,255), new Color(0,0,0));
        SmartDashboard.putString("SettingColorSensor", "Put the color sensor 3-4 inches above the yellow color");
        SmartDashboard.putString("SettingColorSensor", "Now press and hold the red button");
    }

    //CALIBRATORS//
    public static void setBlackColor() {
        SensorManager.colorSensorCalibrate_Black = Hardware.m_colorSensor.getColorSensorV3().getColor();
    }

    public static void setWhiteColor() {
        SensorManager.colorSensorCalibrate_White = Hardware.m_colorSensor.getColorSensorV3().getColor();
    }

    public static void calibrateColor() {

        Hardware.m_colorSensor.calibrateFunctions(SensorManager.colorSensorCalibrate_White, SensorManager.colorSensorCalibrate_Black);

    }

}