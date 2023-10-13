package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp()
public class variablesRB extends OpMode {


    @Override
    public void init() {
        int teamNumber = 16072;
        double motorSpeed = 0.5;
        boolean touchSensorPressed = true;
        telemetry.addData("Team Number", teamNumber);
        telemetry.addData("Motor Speed", motorSpeed);
        telemetry.addData("Touch Sensor", touchSensorPressed);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    }


    public void loop(){
        if (gamepad1.left_stick_y < -0.5) {
            telemetry.addData("Left stick", " is negative and large");
        }
        else if (gamepad1.left_stick_y < 0) {
            telemetry.addData("Left stick", " is negative and small");
        }
        else if (gamepad1.left_stick_y < 0.5) {
            telemetry.addData("Left stick", " is positive and small");
        }
        else {
            telemetry.addData("Left stick", " is positive and large");
        }
        telemetry.addData("Left stick y", gamepad1.left_stick_y);




        }


}
