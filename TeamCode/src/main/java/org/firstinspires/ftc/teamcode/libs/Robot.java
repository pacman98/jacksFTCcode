package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.configuration.LegacyModuleControllerConfiguration;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;

/**
 * Sets up hardware
 */
public class Robot {

    // Reference to the hardware map
    public HardwareMap hardwareMap;

    // Reference to the device interface module
    public DeviceInterfaceModule dim;

    // References to the motor controller
    public DcMotorController ctrlDriveMotor, ctrlFlyWheelMotor, ctrlActuatorMotor;
    public ServoController ctrlWheelServo, ctrlArmServo;
    public LegacyModule ctrlActuatorMotorLegacy;
//    public LegacyModuleControllerConfiguration ctrlActuatorMotor;

    // References to the different motors and servos
    public DcMotor motorDriveLeft, motorDriveRight, motorFlyLeft, motorFlyRight, motorIntakeElevator, motorLift;
    public Servo servoFlyAngle, servoLeftWheel, servoRightWheel, servoLeftArm, servoRightArm,
    servoFlip, servoFeed, servoShoulder, servoElbow;

    // References to sensors
    public ColorSensor colorLine, colorBeacon;
    public ModernRoboticsI2cGyro gyro;
    public UltrasonicSensor rigtSonar;
    public UltrasonicSensor leftSonar;
    public OpticalDistanceSensor lineDetector;
    public ColorSensor colorLeft;
    public ColorSensor colorRight;


    /**
     * Robot class constructor
     * Assigns the hardware references to the software variables
     * @param hwMap Reference to the hardware map
     */
    public Robot(HardwareMap hwMap) {
        hardwareMap = hwMap;

        dim = hardwareMap.deviceInterfaceModule.get("dim");

        ctrlDriveMotor = hardwareMap.dcMotorController.get("drive_motor_ctrl");
        ctrlFlyWheelMotor = hardwareMap.dcMotorController.get("fly_motor_ctrl");

        ctrlActuatorMotorLegacy = hardwareMap.legacyModule.get("actuator_motor_legacy");
        ctrlActuatorMotor = hardwareMap.dcMotorController.get("actuator_motor_ctrl");

        ctrlWheelServo = hardwareMap.servoController.get("wheel_servo_ctrl");
        ctrlArmServo = hardwareMap.servoController.get("arm_servo_ctrl");

        motorDriveLeft = hardwareMap.dcMotor.get("drive_left");
        motorDriveRight = hardwareMap.dcMotor.get("drive_right");
        motorDriveRight.setDirection(DcMotor.Direction.REVERSE);

        motorFlyLeft = hardwareMap.dcMotor.get("fly_left");
        motorFlyRight = hardwareMap.dcMotor.get("fly_right");

        motorIntakeElevator = hardwareMap.dcMotor.get("intake_elevator");
        motorLift = hardwareMap.dcMotor.get("lift");

        servoFlyAngle = hardwareMap.servo.get("fly_angle");
        servoLeftWheel = hardwareMap.servo.get("wheel_left");
        servoRightWheel = hardwareMap.servo.get("wheel_right");
        servoFeed = hardwareMap.servo.get("intake_wheel");

        servoLeftArm = hardwareMap.servo.get("left_arm");
        servoRightArm = hardwareMap.servo.get("right_arm");
//        servoFlip = hardwareMap.servo.get("flip");

        servoShoulder = hardwareMap.servo.get("shoulder");
        servoElbow = hardwareMap.servo.get("elbow");

//        colorLine = hardwareMap.colorSensor.get("color_line");
        colorBeacon = hardwareMap.colorSensor.get("color_beacon");
        //initalazing ultrasonic sensors
        rigtSonar = hardwareMap.ultrasonicSensor.get("rightSonar");
        leftSonar = hardwareMap.ultrasonicSensor.get("leftSonar");
        //light sensor in the buttom
        lineDetector = hardwareMap.opticalDistanceSensor.get("lineDetector");
        colorLeft = hardwareMap.colorSensor.get("colorLeft");
        colorRight = hardwareMap.colorSensor.get("colorRight");

        //getting gyro
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
    }

    //any other hardware methods go here

    public void setArms(double position) {
//        servoLeftArm.setPosition(position);
//        servoRightArm.setPosition(position);
    }
}

