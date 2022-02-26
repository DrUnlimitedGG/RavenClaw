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

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opencv.DuckDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="RedCarousel", group = "Blue")
// @Disabled
public class RedCarousel extends LinearOpMode {

    private DcMotorEx RF;
    private DcMotorEx LF;
    private DcMotorEx RB;
    private DcMotorEx LB;
    private DcMotorEx viper_direct = null;
    private DcMotorEx viper_indirect = null;
    private DcMotorEx carousel;
    private DcMotorEx intake_spinner;
    private DigitalChannel intake_touch;
    private Servo intake_transfer;

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private double encoderConstant = 89.1267682333;
    private double encoderViper = 294;

    public boolean xAxisLockLoop = false;
    public boolean yAxisLockLoop = false;

    private final double speed = 1500;

    private double directPower = 0;

    public static PIDCoefficients pidCoeffsDir = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsDir = new PIDCoefficients(0, 0, 0);
    ElapsedTime PIDTimerDir = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static PIDCoefficients pidCoeffsIndir = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsIndir = new PIDCoefficients(0, 0, 0);
    ElapsedTime PIDTimerIndir = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private String location = "";
    private String ASHLevel;

    private final double carouselConstant = 45.8366237;

    OpenCvCamera webCam;
    WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
    @Override
    public void runOpMode() throws InterruptedException {
        RF = hardwareMap.get(DcMotorEx.class, "right_front");
        LF = hardwareMap.get(DcMotorEx.class, "left_front");
        RB = hardwareMap.get(DcMotorEx.class, "right_back");
        LB = hardwareMap.get(DcMotorEx.class, "left_back");
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        intake_touch = hardwareMap.get(DigitalChannel.class, "intake_touch");

        intake_spinner = hardwareMap.get(DcMotorEx.class, "intake_spinner");
        intake_transfer = hardwareMap.get(Servo.class, "intake_transfer");
        viper_direct = hardwareMap.get(DcMotorEx.class, "viper_direct");
        viper_indirect = hardwareMap.get(DcMotorEx.class, "viper_indirect");

        viper_direct.setDirection(DcMotorEx.Direction.REVERSE);
        RF.setDirection(DcMotorEx.Direction.REVERSE);
        RB.setDirection(DcMotorEx.Direction.REVERSE);
        LF.setDirection(DcMotorEx.Direction.FORWARD);
        LB.setDirection(DcMotorEx.Direction.FORWARD);
        viper_indirect.setDirection(DcMotorEx.Direction.FORWARD);

        carousel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        intake_spinner.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake_spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        viper_direct.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        viper_indirect.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            telemetry.addData("Running: ", "Blue Right");
            telemetry.update();
            // MOVEMENT HERE


            //OPENCV
            /*int cameraMonitorViewId = hardwareMap.appContext
                    .getResources().getIdentifier("cameraMonitorViewId",
                            "id", hardwareMap.appContext.getPackageName());
            webCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
            DuckDetector detector = new DuckDetector(telemetry);
            webCam.setPipeline(detector);
            webCam.openCameraDeviceAsync(
                    new OpenCvCamera.AsyncCameraOpenListener() {
                        @Override
                        public void onOpened() {
                            webCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
                        }

                        @Override
                        public void onError(int errorCode) {

                        }
                    }
            );

            switch (detector.getLocation()) {
                case LEFT:
                    location = "LEFT";
                    telemetry.addData("Shipping Hub: ", "Bottom");
                    break;
                case RIGHT:
                    location = "RIGHT";
                    telemetry.addData("Shipping Hub: ", "Top");
                    break;
                case MIDDLE:
                    location = "MIDDLE";
                    telemetry.addData("Shipping Hub: ", "Middle");
                case NOT_FOUND:
                    location = "NONE";
                    telemetry.addData("Shipping Hub: ", "Not found!");
            }
            webCam.stopStreaming();

            //move to shipping hub
             */
            //if statements for location string var
            //VIPER SLIDE FREIGHT CODE HERE

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    public void quitDriving() {
        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
    }

    public void BottomSH() {

    }

    public void MiddleSH() {

    }

    public void TopSH() {

    }

    public void spinCarousel(double distance, double speed) {
        carousel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setTargetPosition((int) (distance * carouselConstant));
        carousel.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        carousel.setVelocity(speed);

        while (carousel.isBusy()) {
            idle();
        }

        carousel.setPower(0);
    }

    public void spinCarouselBackwards(double distance, double speed) {
        carousel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setTargetPosition((int) (-1 * (distance * carouselConstant)));
        carousel.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        carousel.setVelocity(speed);

        while (carousel.isBusy()) {
            idle();
        }

        carousel.setPower(0);
    }

    public void driveForward(double distance, double speed) {
        LF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        LF.setTargetPosition((int) (distance * encoderConstant));
        LB.setTargetPosition((int) (distance * encoderConstant));
        RF.setTargetPosition((int) (distance * encoderConstant));
        RB.setTargetPosition((int) (distance * encoderConstant));

        LF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        LF.setVelocity(speed);
        LB.setVelocity(speed);
        RF.setVelocity(speed);
        RB.setVelocity(speed);

        while (LF.isBusy() && LB.isBusy() && RF.isBusy() && RB.isBusy()) {
            idle();
        }

        quitDriving();

    }

    public void driveBack(double distance, double speed) {
        LF.setTargetPosition((int) (-1 * (distance * encoderConstant)));
        LB.setTargetPosition((int) (-1 * (distance * encoderConstant)));
        RF.setTargetPosition((int) (-1 * (distance * encoderConstant)));
        RB.setTargetPosition((int) (-1 * (distance * encoderConstant)));

        LF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        LF.setVelocity(speed);
        LB.setVelocity(speed);
        RF.setVelocity(speed);
        RB.setVelocity(speed);

        while (LF.isBusy() && LB.isBusy() && RF.isBusy() && RB.isBusy()) {
            idle();
        }

        quitDriving();
    }

    public void turnRight(double distance, double speed) {
        RF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        RF.setTargetPosition((int) (-1 * (distance * encoderConstant)));
        RB.setTargetPosition((int) (-1 * (distance * encoderConstant)));
        LF.setTargetPosition((int) (distance * encoderConstant));
        LB.setTargetPosition((int) (distance * encoderConstant));

        RF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        RF.setVelocity(-1 * speed);
        RB.setVelocity(-1 * speed);
        LF.setVelocity(speed);
        LB.setVelocity(speed);

        while (RF.isBusy() || RB.isBusy() || LF.isBusy() || LB.isBusy()) {
            idle();
        }

        quitDriving();
    }

    public void turnLeft(double distance, double speed) {
        RF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        RF.setTargetPosition((int) (distance * encoderConstant));
        RB.setTargetPosition((int) (distance * encoderConstant));
        LF.setTargetPosition((int) (-1 * (distance * encoderConstant)));
        LB.setTargetPosition((int) (-1 * (distance * encoderConstant)));

        RF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        RF.setVelocity(speed);
        RB.setVelocity(speed);
        LF.setVelocity(-1 * speed);
        LB.setVelocity(-1 * speed);

        while (RF.isBusy() || RB.isBusy() || LF.isBusy() || LB.isBusy()) {
            idle();
        }

        quitDriving();
    }



    public void carouselSpin(double time) throws InterruptedException {
        carousel.setVelocity(0.25);
        Thread.sleep((long) (time * 1000));
        carousel.setVelocity(0);
    }

    public void viper_extend() {
        viper_direct.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        viper_indirect.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        viper_direct.setTargetPosition(1569);
        viper_indirect.setTargetPosition(1569);

        viper_direct.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        viper_indirect.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        viper_direct.setVelocity(speed);
        viper_indirect.setVelocity(speed);

        while (viper_direct.isBusy() && viper_indirect.isBusy()) {
            PIDdirect(speed);
            directPower = viper_direct.getVelocity();
            PIDindirect(directPower);

            telemetry.addData("Direct: ", viper_direct.getVelocity());
            telemetry.addData("Indirect: ", viper_indirect.getVelocity());
            telemetry.update();

        }

        viper_direct.setPower(0);
        viper_indirect.setPower(0);

    }

    public void viper_lower() {
        /*viper_direct.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        viper_indirect.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);*/

        viper_direct.setTargetPosition(-10);
        viper_indirect.setTargetPosition(-10);

        viper_direct.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        viper_indirect.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        viper_direct.setVelocity(-speed);
        viper_indirect.setVelocity(-speed);

        while (viper_direct.isBusy() && viper_indirect.isBusy()) {
            PIDdirect(-speed);
            directPower = viper_direct.getVelocity();
            PIDindirect(directPower);
        }

        viper_direct.setPower(0);
        viper_indirect.setPower(0);

    }

    double integralDir = 0;
    double lastErrorDir = 0;
    public void PIDdirect(double targetVelocity) {
        PIDTimerDir.reset();

        double currentVelocity = viper_direct.getVelocity();

        double error = targetVelocity - currentVelocity;

        integralDir += error * PIDTimerDir.time();

        double deltaError = error - lastErrorDir;
        double derivative = deltaError / PIDTimerDir.time();

        pidGainsDir.p = pidCoeffsDir.p * error;
        pidGainsDir.i = pidCoeffsDir.i * integralDir;
        pidGainsDir.d = pidCoeffsDir.d * derivative;

        viper_direct.setVelocity(targetVelocity + pidGainsDir.p + pidGainsDir.i + pidGainsDir.d);

        lastErrorDir = error;
    }

    double integralIndir = 0;
    double lastErrorIndir = 0;
    public void PIDindirect(double targetVelocity) {
        PIDTimerIndir.reset();

        double currentVelocity = viper_indirect.getVelocity();

        double error = targetVelocity - currentVelocity;

        integralDir += error * PIDTimerIndir.time();

        double deltaError = error - lastErrorDir;
        double derivative = deltaError / PIDTimerIndir.time();

        pidGainsIndir.p = pidCoeffsIndir.p * error;
        pidGainsIndir.i = pidCoeffsIndir.i * integralDir;
        pidGainsIndir.d = pidCoeffsIndir.d * derivative;

        viper_indirect.setVelocity(targetVelocity + pidGainsIndir.p + pidGainsIndir.i + pidGainsIndir.d);

        lastErrorDir = error;
    }
}
