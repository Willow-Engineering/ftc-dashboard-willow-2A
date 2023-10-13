package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="gamepadtestingRB")
public class gamepadtestingRB extends OpMode {
    double left_joystic_y;
    boolean button_pressed;
    double forward_speed;
    private DcMotor motor1;


    @Override
    public void init() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        motor1  = hardwareMap.get(DcMotor.class, "motor1");


    }


    public void loop() {
        left_joystic_y = gamepad1.left_stick_y;
        button_pressed = gamepad1.a;

        if (button_pressed) {
            forward_speed = forward_speed * 2;
        }
        //run the motor
        if (gamepad1.b){
            motor1.setPower(1);



        }


    }
}
