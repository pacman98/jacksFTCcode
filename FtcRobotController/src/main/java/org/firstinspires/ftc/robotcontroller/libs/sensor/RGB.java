package org.firstinspires.ftc.robotcontroller.libs.sensor;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/**
 * Created by Jack & Natalie on 11/6/16.
 * RGB Sensor Library Class
 */
public class RGB {

    // References to the hardware devices
    private static ColorSensor rgbSensor;
    private static DeviceInterfaceModule cdim;

    // HSV value array
    private static float hsvValues[] = {0F,0F,0F};

    // The pin that the sensor LED is connected to
    static final int LED_CHANNEL = 5;

    // BASELINE colors for the sensor
    private static int BASELINE_RED;
    private static int BASELINE_BLUE;

    /**
     * Retrieves references to the sensor and device interface module
     * Sets up and turns on sensor's LED
     * Default baseline values until calibration
     * @param sensor Reference to sensor hardware
     * @param dim Reference to the device interface module hardware
     */
    public static void init(ColorSensor sensor, DeviceInterfaceModule dim) {
        rgbSensor = sensor;
        cdim = dim;

        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        cdim.setDigitalChannelState(LED_CHANNEL, true);

        BASELINE_BLUE = 127;
        BASELINE_RED = 127;
    }

    /**
     * Retrieves references to the sensor and device interface module
     * Sets up and turns on sensor's LED
     * Sets baseline values until calibration
     * @param sensor Reference to the sensor hardware
     * @param dim Reference to the device interface module
     * @param baseRed Baseline value for red
     * @param baseBlue Baseline value for blue
     */
    public static void init(ColorSensor sensor, DeviceInterfaceModule dim, int baseRed, int
            baseBlue) {
        rgbSensor = sensor;
        cdim = dim;

        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        cdim.setDigitalChannelState(LED_CHANNEL, true);

        BASELINE_BLUE = baseBlue;
        BASELINE_RED = baseRed;
    }

    /**
     * Calibrates the sensors
     * Experiemental
     * @return Nothing currently
     */
    public static int[] calibrate() {
        return null;
    }

    /**
     * Retrieves the color value from the sensor and checks against baseline
     * @param color The desired color name
     * @return true is value is above baseline, false if not
     */
    public static boolean getSensorValue(String color) {
        switch(color) {
            case "red":
                int red = rgbSensor.red();
                if (red >= BASELINE_RED) {
                    return true;
                }
                break;
            case "blue":
                int blue = rgbSensor.blue();
                if (blue >= BASELINE_BLUE) {
                    return true;
                }
                break;
            default:
                break;
        }
        return false;
    }

    /**
     * Transforms the current reading of the sensor into a hexadecimal value
     * @return The array of hex values
     */
    public static float[] normalize() {
        Color.RGBToHSV((rgbSensor.red() * 255) / 800, (rgbSensor.green() * 255) / 800, (rgbSensor
                .blue() * 255) / 800, hsvValues);
        return hsvValues;
    }

    /**
     * Sets the state of the LED
     * @param state Boolean, on (true) or off (false)
     */
    public static void setLED(boolean state) {
        cdim.setDigitalChannelState(LED_CHANNEL, state);
    }
}
