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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="BlueRight", group = "Blue")
public class BlueRight extends LinearOpMode {

    // Declare OpMode members.
    //dadad
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private DcMotorEx left_front = null;
    private DcMotorEx left_back = null;
    private DcMotorEx right_front = null;
    private DcMotorEx right_back = null;
    private DcMotorEx carousel = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Stat", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        left_front  = hardwareMap.get(DcMotorEx.class, "left_front");
        left_back = hardwareMap.get(DcMotorEx.class, "left_back");
        right_front = hardwareMap.get(DcMotorEx.class, "right_front");
        right_back = hardwareMap.get(DcMotorEx.class, "right_back");
        carousel = hardwareMap.get(DcMotorEx.class, "sustainable");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        left_front.setDirection(DcMotorEx.Direction.REVERSE);
        left_back.setDirection(DcMotorEx.Direction.REVERSE);
        right_front.setDirection(DcMotorEx.Direction.FORWARD);
        right_back.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            telemetry.addData("Running: ", "Blue Right");
            telemetry.update();

            driveForward(0.5, 1);
            Thread.sleep(100);
            //turnLeft(0.5, 4);

            turnLeft(0.5, 2);
            Thread.sleep(100);

            strafeRight(0.5, 0.4);
            Thread.sleep(100);

            driveForward(0.5, 0.9);
            Thread.sleep(100);

            spinCarousel(0.2, 4.5);
            Thread.sleep(100);

            driveBack(0.5, 0.9);
            Thread.sleep(100);

            turnLeft(0.5, 0.25);
            Thread.sleep(100);

            driveForward(0.5, 0.5);
            
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    public void quitDriving() {
        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
    }

    public void driveForward(double speed, double time) throws InterruptedException {
        left_front.setPower(-speed);
        left_back.setPower(-speed);
        right_front.setPower(-speed);
        right_back.setPower(-speed);

        double sleepTime = time * 1000;
        Thread.sleep((long) sleepTime);

        quitDriving();
    }

    public void driveBack(double speed, double time) throws InterruptedException {
        left_front.setPower(speed);
        left_back.setPower(speed);
        right_front.setPower(speed);
        right_back.setPower(speed);

        double sleepTime = time * 1000;
        Thread.sleep((long) sleepTime);

        quitDriving();
    }

    public void turnLeft(double speed, double time) throws InterruptedException {
        left_front.setPower(-speed);
        left_back.setPower(-speed);
        right_front.setPower(speed);
        right_back.setPower(speed);

        double sleepTime = time * 1000;
        Thread.sleep((long) sleepTime);

        quitDriving();
    }

    public void turnRight(double speed, double time) throws InterruptedException {
        left_front.setPower(speed);
        left_back.setPower(speed);
        right_front.setPower(-speed);
        right_back.setPower(-speed);

        double sleepTime = time * 1000;
        Thread.sleep((long) sleepTime);

        quitDriving();
    }

    public void spinCarousel(double speed, double time) throws InterruptedException {
        carousel.setPower(speed);

        double sleepTime = time * 1000;
        Thread.sleep((long) sleepTime);

        carousel.setPower(0);
    }

    public void strafeRight(double speed, double time) throws InterruptedException {
        left_front.setPower(-speed);
        left_back.setPower(speed);
        right_front.setPower(speed);
        right_back.setPower(-speed);

        double sleepTime = time * 1000;
        Thread.sleep((long) sleepTime);

        quitDriving();
    }

    public void strafeLeft(double speed, double time) throws InterruptedException {
        left_front.setPower(speed);
        left_back.setPower(-speed);
        right_front.setPower(-speed);
        right_back.setPower(speed);

        double sleepTime = time * 1000;
        Thread.sleep((long) sleepTime);

        quitDriving();
    }
}
