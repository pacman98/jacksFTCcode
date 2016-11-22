package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.libs.MotorFunctions;
import org.firstinspires.ftc.robotcontroller.libs.sensor.RGB;
import org.firstinspires.ftc.teamcode.libs.Robot;

/**
 *
 * Created by Jack Packham on 11/22/16
 */
@Autonomous(name="Red 2 Beacon Overlord", group="Red Autonomous")
//@Disabled
public class RedBeaconBeacon extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot;
    private MotorFunctions motorFunctions;
    private RGB colorLine, colorBeacon;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot = new Robot(hardwareMap);
        motorFunctions = new MotorFunctions(-1, 1, 0, 1, .1);
        colorLine.init(robot.colorLine, robot.dim);
        colorBeacon.init(robot.colorBeacon, robot.dim);


        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) { //change to if statement, if it runs more than once.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            while (!colorLine.getSensorValue("white")) {
                robot.motorDriveLeft.setPower(-.85);
                robot.motorDriveRight.setPower(-.85);
            }
            robot.motorDriveLeft.setPower(0);
            robot.motorDriveRight.setPower(0);

            sleep(250);

            robot.motorDriveLeft.setPower(.85);
            robot.motorDriveRight.setPower(-.85);

            sleep(500);

            robot.motorDriveLeft.setPower(-.85);
            robot.motorDriveRight.setPower(-.85);

            sleep(250);

            if (colorBeacon.getSensorValue("red")) {
                //actuate on the button
            } else {
                //actuate on the other button
            }

            //insert next becon/shoot
        }
    }
}
