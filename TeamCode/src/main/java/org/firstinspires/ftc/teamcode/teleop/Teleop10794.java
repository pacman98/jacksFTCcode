package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.libs.MotorFunctions;

import org.firstinspires.ftc.teamcode.libs.Robot;

import java.text.SimpleDateFormat;
import java.util.Date;

@TeleOp(name="10794 Teleop", group="Iterative Opmode")  // @Autonomous(...) is the other common choice
//@Disabled
public class Teleop10794 extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private Robot robot;
    private MotorFunctions motorFunctions;

    private float joystick1L_y, joystick2L_y, joystick1R_y, joystick2R_y;

    private boolean servoStateArms, servoStateFlip;
    private double servoPositionFlyAngle;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        //may wish to pass in telemetry to add telemtry data
        /**
         * Initializes the library functions
         * Robot hardware and motor functions
         */
        robot = new Robot(hardwareMap);
        motorFunctions = new MotorFunctions(-1, 1, 0, 1, .1);
        //set servo position either or in robot
    }

    /*
    * Code to run REPEATEDLY when the driver hits INIT
    * We don't need this method
    */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     * We don't need this method
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Assigns the gamepad buttons to the motor and servos
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        /**
         * Sets the power level of the drive motors to the joystick values
         */
        robot.motorDriveLeft.setPower(MotorFunctions.dcMotor(gamepad1.left_stick_y));
        robot.motorDriveRight.setPower(MotorFunctions.dcMotor(gamepad1.right_stick_y));

        /**
         * Gamepad 1
         */
        if (gamepad1.a) {
            //beacon
        }
        if (gamepad1.b) {
            //beacon
        }
        if (gamepad1.x) {
            //beacon
        }
        if (gamepad1.y) {
            //beacon
        }
        if (gamepad1.dpad_up) {
            //not in use
        }
        if (gamepad1.dpad_down) {
            //not in use
        }
        if (gamepad1.dpad_left) {
            //not in use
        }
        if (gamepad1.dpad_right) {
            //not in use
        }
        if (gamepad1.left_bumper) {
            //not in use
        }
        if (gamepad1.right_bumper) {
            //not in use
        }
        if (gamepad1.left_trigger > 0.25) {
            //not in use
        }
        if (gamepad1.right_trigger > 0.25) {
            //not in use
        }

        /**
         * Gamepad 2
         */
        if (gamepad2.a) {
            servoStateArms = !servoStateArms;
        }
        if (gamepad2.b) {
            servoStateFlip = !servoStateFlip;
        }
        if (gamepad2.x) {
            //Automatic fire -- separate targeting class
        }
        if (gamepad2.y) {
            robot.motorIntakeElevator.setPower(0);
        }
        if (gamepad2.dpad_up) {
            robot.motorIntakeElevator.setPower(.85);
        }
        if (gamepad2.dpad_down) {
            robot.motorIntakeElevator.setPower(-.85);
        }
        if (gamepad2.dpad_left) {
            //not in use
        }
        if (gamepad2.dpad_right) {
            //not in use
        }
        if (gamepad2.left_bumper) {
            servoPositionFlyAngle = MotorFunctions.servoDecrement(servoPositionFlyAngle);
        }
        if (gamepad2.right_bumper) {
            servoPositionFlyAngle = MotorFunctions.servoIncrement(servoPositionFlyAngle);
        }
        if (gamepad2.left_trigger > 0.1) {
            robot.motorFlyLeft.setPower(gamepad2.left_trigger);
        }
        if (gamepad2.right_trigger > 0.1) {
            robot.motorFlyRight.setPower(gamepad2.right_trigger);
        }
        if (gamepad2.left_stick_y > 0.25) {
            servoPositionFlyAngle = MotorFunctions.servoIncrement(servoPositionFlyAngle);
        }

        /**
         * Set robot servos to binary values
         */
        robot.setArms(servoStateArms ? MotorFunctions.servo(1) : MotorFunctions.servo(0));
        robot.servoFlip.setPosition(servoStateFlip ? MotorFunctions.servo(1) : MotorFunctions.servo(0));

        /**
         * Sets the position for the manual fly wheel
         */
        robot.servoFlyAngle.setPosition(servoPositionFlyAngle);
    }

    /**
     * Code to run ONCE after STOP is pressed
     * DO NOT USE THIS METHOD!!
     */
    @Override
    public void stop() {

    }


}
