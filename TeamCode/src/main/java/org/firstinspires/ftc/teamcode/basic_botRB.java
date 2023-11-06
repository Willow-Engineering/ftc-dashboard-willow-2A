/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




@TeleOp(name="basic_botRB")
@Config
//@Disabled
public class basic_botRB extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx armDrive = null;
    private Servo leftServo = null;
    private Servo rightServo = null;
    public static int armTarget=-700;
    public static int armVelocity=200;
    public static int armReset=-45;
    public static int armSpeed=200;
    public static double leftClawOpen = -0.5;
    public static double rightClawOpen = 0.5;
    public static double reset = 0;






    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();




        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        leftServo  = hardwareMap.get(Servo.class, "left_servo");
        rightServo  = hardwareMap.get(Servo.class, "right_servo");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armDrive = hardwareMap.get(DcMotorEx.class,  "arm_drive");



          leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // add ex after dc motor maybe


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;





            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;



            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;



            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            if (gamepad1.a){

                armDrive.setTargetPosition(armTarget);

                armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                armDrive.setVelocity (armVelocity);

            } else if (gamepad1.b) {

                armDrive.setTargetPosition(armReset);

                armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                armDrive.setVelocity(armSpeed);

            }
            if (gamepad1.x) {
                leftServo.setPosition(leftClawOpen);
                rightServo.setPosition(rightClawOpen);
            } else if (gamepad1.y) {
                rightServo.setPosition(reset);
                leftServo.setPosition(reset);


            }



            telemetry.addData("velocity",armDrive.getVelocity());
            telemetry.addData("position",armDrive.getCurrentPosition());
            telemetry.addData("position1",leftServo.getPosition());
            telemetry.addData("position2",rightServo.getPosition());

            telemetry.update();








            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
